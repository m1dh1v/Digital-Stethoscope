%% live_fft.m
% Live microphone signal, spectrum, and spectrogram from the ESP32.
% Run in MATLAB, not the Arduino IDE. Requires Signal Processing Toolbox.
% Uses live_fft_stream.ino (D35, 2000 Hz, 115200 baud).
% Close Arduino's Serial Monitor/Plotter first. Close the figure to stop.

clear; close all; clc;

PORT   = "COM10";    % ESP32 port
BAUD   = 115200;    % Must match SERIAL_BAUD in the sketch
N      = 64;        % Must match FRAME_VALUES in the sketch
FS_NOM = 2000;      % Must match SAMPLE_RATE_HZ in the sketch
FMAX   = 300;       % Highest frequency shown, in Hz

liveFFT(PORT, BAUD, N, FS_NOM, FMAX);

%% ---------------------------------------------------------------- local functions
function liveFFT(PORT, BAUD, N, FS_NOM, FMAX)
    SEG      = 1024;      % FFT length: about 0.5 s, 2 Hz resolution
    HIST_S   = 4;         % Seconds used for the averaged spectrum
    SPEC_S   = 30;        % Seconds shown in the spectrogram
    UPDATE_S = 0.25;      % Screen update interval

    s = serialport(PORT, BAUD);
    s.Timeout = 2;
    guard = onCleanup(@() stopStream(s)); %#ok<NASGU>
    pause(2.5);                          % The ESP32 usually resets when the port opens
    flush(s);
    write(s, uint8('s'), "uint8");       % Start streaming

    L    = round(HIST_S * FS_NOM);
    sig  = nan(L, 1);                    % Recent signal samples
    have = 0;
    st   = newState();
    fs   = FS_NOM;

    fAxis = (0:SEG/2)' * (FS_NOM / SEG);
    fsel  = fAxis <= FMAX;
    nCols = round(SPEC_S / UPDATE_S);
    M     = nan(nnz(fsel), nCols);

    fig = figure("Name", "Live spectrum", "Color", "w", "Position", [80 60 1000 850]);
    tiledlayout(3, 1, "TileSpacing", "compact");
    ax1 = nexttile; h1 = plot(ax1, nan, nan); grid(ax1, "on");
    xlim(ax1, [-HIST_S 0]); xlabel(ax1, "s"); ylabel(ax1, "counts");
    title(ax1, "Signal, last 4 s (mean removed)");
    ax2 = nexttile; h2 = plot(ax2, nan, nan); grid(ax2, "on");
    xlim(ax2, [0 FMAX]); xline(ax2, 50, ":"); xline(ax2, 100, ":");
    xlabel(ax2, "Hz"); ylabel(ax2, "dB (counts^2/Hz)"); title(ax2, "Collecting data ...");
    ax3 = nexttile;
    img = imagesc(ax3, [-SPEC_S 0], fAxis(fsel), M); axis(ax3, "xy"); colorbar(ax3);
    xlabel(ax3, "s"); ylabel(ax3, "Hz"); title(ax3, "Spectrogram (bright = more energy)");

    tUpd = tic;
    while ishandle(fig)
        [new, st] = pullSamples(s, st, N);
        if ~isempty(new)
            have = min(L, have + numel(new));
            if numel(new) >= L
                sig = new(end-L+1:end);
            else
                sig = [sig(numel(new)+1:end); new];
            end
        end
        if st.frames >= 30 && isfinite(st.fs)
            fs = st.fs;                  % Use the sample rate measured from ESP32 timestamps
        end

        if toc(tUpd) >= UPDATE_S && have >= SEG
            tUpd = tic;
            x  = fillmissing(sig(end-have+1:end), "linear");   % Fill gaps from dropped blocks
            xz = x - mean(x);
            [px, f] = pwelch(xz, hann(SEG), SEG/2, SEG, fs);
            pc      = pwelch(xz(end-SEG+1:end), hann(SEG), 0, SEG, fs);

            sel = f <= FMAX;
            set(h2, "XData", f(sel), "YData", 10*log10(px(sel) + eps));
            band = f >= 15 & f <= 80;
            fb = f(band); [~, ib] = max(px(band));
            title(ax2, sprintf("Averaged spectrum, last %d s. Strongest peak in 15-80 Hz: %.1f Hz   (fs = %.1f Hz)", ...
                               HIST_S, fb(ib), fs));

            set(h1, "XData", (-(numel(xz)-1):0)'/fs, "YData", xz);

            M = [M(:, 2:end), 10*log10(pc(1:size(M, 1)) + eps)];
            set(img, "CData", M);
            v = M(isfinite(M));
            if numel(v) > 10 && max(v) > median(v)
                caxis(ax3, [median(v) max(v)]);
            end
            drawnow limitrate
        end
        pause(0.01);
    end
end

function stopStream(s)
    try
        write(s, uint8('x'), "uint8");   % Tell the ESP32 to stop
    catch
    end
end

function st = newState()
    st = struct("buf", zeros(0, 1, "uint8"), "seqPrev", NaN, "frames", 0, "idx", 0, ...
                "idx0", NaN, "t0", NaN, "tLast", NaN, "fs", NaN);
end

function [x, st] = pullSamples(s, st, N)
    % Read complete valid frames from the serial buffer.
    % Frame: AA 55 | seq u16 | t_us u32 | n u16 | missed u16 | n x u16 | sum16 u16 (little-endian)
    nb = s.NumBytesAvailable;
    if nb > 0
        st.buf = [st.buf; read(s, nb, "uint8")'];
    end
    FR  = 14 + 2*N;
    raw = st.buf;
    L   = numel(raw);
    pos = 1;
    xs  = {};
    while pos + FR - 1 <= L
        if raw(pos) == 170 && raw(pos+1) == 85
            f      = raw(pos:pos+FR-1);
            nField = double(typecast(f(9:10), 'uint16'));
            calc   = mod(sum(double(f(3:FR-2))), 65536);
            got    = double(typecast(f(FR-1:FR), 'uint16'));
            if nField == N && calc == got
                seq = double(typecast(f(3:4), 'uint16'));
                tus = double(typecast(f(5:8), 'uint32'));
                smp = double(typecast(f(13:12+2*N), 'uint16'));
                if isnan(st.seqPrev)
                    st.idx = 0;
                else
                    step = mod(seq - st.seqPrev, 65536);
                    if step > 1 && step <= 200            % Insert gaps for missed frames
                        xs{end+1} = nan((step-1)*N, 1); %#ok<AGROW>
                    end
                    st.idx = st.idx + step;
                end
                if isnan(st.t0), st.t0 = tus; st.idx0 = st.idx; end
                st.tLast   = tus;
                st.seqPrev = seq;
                st.frames  = st.frames + 1;
                xs{end+1}  = smp(:); %#ok<AGROW>
                pos = pos + FR;
                continue
            end
        end
        pos = pos + 1;
    end
    st.buf = raw(pos:end);                                    % Keep any incomplete frame for next read
    if st.frames >= 30
        dt = mod(st.tLast - st.t0, 2^32) * 1e-6;
        if dt > 0, st.fs = N * (st.idx - st.idx0) / dt; end
    end
    if isempty(xs), x = zeros(0, 1); else, x = vertcat(xs{:}); end
end
