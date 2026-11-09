%% live_fft.m
% Live frequency view of the microphone on D35: signal, averaged spectrum, and a scrolling spectrogram.
%
% Run this in MATLAB (NOT in the Arduino IDE).
% Needs: MATLAB R2019b or newer, Signal Processing Toolbox.
% Sketch on the ESP32: live_fft_stream.ino  (1 channel, 2000 Hz, 115200 baud).
% Close the Arduino Serial Monitor and Serial Plotter first (only one program can hold the port).
% Close the figure window to stop. The port is released automatically.

clear; close all; clc;

PORT   = "COM10";    % <-- your ESP32 port (Arduino IDE > Tools > Port)
BAUD   = 115200;    % MUST equal SERIAL_BAUD in the sketch
N      = 64;        % MUST equal FRAME_VALUES in the sketch
FS_NOM = 2000;      % MUST equal SAMPLE_RATE_HZ in the sketch
FMAX   = 300;       % highest frequency shown, in Hz
MAINS_HZ = 50;      % mains frequency of your country (50 or 60)

liveFFT(PORT, BAUD, N, FS_NOM, FMAX, MAINS_HZ);

%% ---------------------------------------------------------------- local functions
function liveFFT(PORT, BAUD, N, FS_NOM, FMAX, MAINS_HZ)
    SEG      = 1024;      % FFT length: about 0.5 s, 2 Hz resolution
    HIST_S   = 4;         % seconds averaged in the spectrum
    SPEC_S   = 30;        % seconds shown in the spectrogram
    UPDATE_S = 0.25;      % seconds between screen updates

    s = serialport(PORT, BAUD);          % opened INSIDE this function so it is always closed again
    s.Timeout = 2;
    guard = onCleanup(@() stopStream(s)); %#ok<NASGU>
    pause(2.5);                          % opening the port usually resets the ESP32: wait for it to boot
    flush(s);
    write(s, uint8('s'), "uint8");       % start streaming

    L    = round(HIST_S * FS_NOM);
    sig  = nan(L, 1);                    % most recent HIST_S seconds
    have = 0;                            % how many valid samples are in sig
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
    title(ax1, "Signal after mains-hum removal, last 4 s");
    ax2 = nexttile;
    h2raw = plot(ax2, nan, nan, "Color", [0.7 0.7 0.7]); hold(ax2, "on");
    h2    = plot(ax2, nan, nan, "Color", [0 0.45 0.74]); hold(ax2, "off");
    grid(ax2, "on"); xlim(ax2, [0 FMAX]);
    legend(ax2, [h2raw h2], {"raw", "hum removed"});
    xlabel(ax2, "Hz"); ylabel(ax2, "dB (counts^2/Hz)"); title(ax2, "Collecting data ...");
    ax3 = nexttile;
    img = imagesc(ax3, [-SPEC_S 0], fAxis(fsel), M); axis(ax3, "xy"); colorbar(ax3);
    xlabel(ax3, "s"); ylabel(ax3, "Hz"); title(ax3, "Spectrogram after hum removal (bright = more energy)");

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
            fs = st.fs;                  % real sample rate measured from the board's timestamps
        end

        if toc(tUpd) >= UPDATE_S && have >= SEG
            tUpd = tic;
            x  = fillmissing(sig(end-have+1:end), "linear");   % bridge any dropped blocks
            xr = x - mean(x);                                  % raw, mean removed
            [xz, f0, humRms] = removeHum(x, fs, MAINS_HZ);     % mains hum and harmonics subtracted
            [pr, f] = pwelch(xr, hann(SEG), SEG/2, SEG, fs);   % spectrum before hum removal
            px      = pwelch(xz, hann(SEG), SEG/2, SEG, fs);   % averaged spectrum after hum removal
            pc      = pwelch(xz(end-SEG+1:end), hann(SEG), 0, SEG, fs); % newest column for the spectrogram

            sel = f <= FMAX;
            set(h2raw, "XData", f(sel), "YData", 10*log10(pr(sel) + eps));
            set(h2,    "XData", f(sel), "YData", 10*log10(px(sel) + eps));
            band = f >= 15 & f <= 80;
            fb = f(band); [~, ib] = max(px(band));
            title(ax2, {sprintf("Mains found at %.2f Hz, hum = %.0f counts rms (raw signal rms = %.0f)", ...
                                f0, humRms, std(xr)), ...
                        sprintf("After hum removal: strongest peak in 15-80 Hz at %.1f Hz   (fs = %.1f Hz)", fb(ib), fs)});

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

function [y, f0, humRms] = removeHum(x, fs, mains)
    % Finds the real mains frequency (it drifts, e.g. 49.7 to 50.3 Hz) and subtracts a fitted copy of the
    % mains hum and its harmonics (up to 7x). A fixed 50.000 Hz notch cannot do this well on a strong hum.
    n  = numel(x);
    xz = x - mean(x);
    t  = (0:n-1)'/fs;
    nf = 2^17;                                    % zero-padded FFT for a fine frequency estimate
    P  = abs(fft(xz .* hann(n), nf));
    P  = P(1:nf/2+1);
    fr = (0:nf/2)' * (fs/nf);
    cand = find(fr >= mains-3 & fr <= mains+3);
    [~, j] = max(P(cand));
    i  = cand(j);
    la = log(P(i-1)); lb = log(P(i)); lc = log(P(i+1));
    f0 = fr(i) + 0.5*(la - lc)/(la - 2*lb + lc) * (fs/nf);   % parabolic refinement of the peak
    K  = 7;
    tt = linspace(-1, 1, n)';
    A  = [ones(n,1), tt];                         % slow drift
    for k = 1:K
        c1 = cos(2*pi*k*f0*t);
        s1 = sin(2*pi*k*f0*t);
        A  = [A, c1, s1, tt.*c1, tt.*s1];         %#ok<AGROW> the tt terms let the hum amplitude change slowly
    end
    coef   = A \ xz;
    y      = xz - A*coef;
    humRms = sqrt(mean((A(:, 3:end)*coef(3:end)).^2));
end

function stopStream(s)
    try
        write(s, uint8('x'), "uint8");   % tell the ESP32 to stop streaming
    catch
    end
end

function st = newState()
    st = struct("buf", zeros(0, 1, "uint8"), "seqPrev", NaN, "frames", 0, "idx", 0, ...
                "idx0", NaN, "t0", NaN, "tLast", NaN, "fs", NaN);
end

function [x, st] = pullSamples(s, st, N)
    % Reads whatever bytes have arrived and returns the samples of all complete, valid frames.
    % Frame: AA 55 | seq u16 | t_us u32 | n u16 | missed u16 | n x u16 | sum16 u16  (little-endian)
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
                    if step > 1 && step <= 200            % blocks were dropped: keep timing with NaNs
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
    st.buf = raw(pos:end);                                    % keep the unfinished frame for next time
    if st.frames >= 30
        dt = mod(st.tLast - st.t0, 2^32) * 1e-6;
        if dt > 0, st.fs = N * (st.idx - st.idx0) / dt; end   % samples per second
    end
    if isempty(xs), x = zeros(0, 1); else, x = vertcat(xs{:}); end
end
