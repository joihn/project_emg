
%%
variables = hri_load_logfile("biceps_haut/florian_poignet.csv");

emg_signal = variables.emg_signal;
emg_signal = emg_signal - mean(emg_signal);
position = variables.paddle_pos__deg_;
%%
figure
threshold = 0
bin_array= position > threshold

plot(bin_array/6)
hold on;
plot(emg_signal)
%%
on_whole= emg_signal(bin_array)
off_whole= emg_signal(~bin_array)
%%
% on_start= 6328
% on_stop= 8372
% 
% 
% off_start= on_stop
% off_stop= 11804
% 
% 
% on_emg = emg_signal(on_start:off_start);
% off_emg = emg_signal(on_stop+1:off_stop-1);
%%

close all
figure
for choice = 0:1
    if choice == 0
    sign = off_whole;
    else
        hold on;
        sign = on_whole;
    end
    plot_sign(choice, sign)
end


%%
function plot_sign(choice, sign)
    
    
    % sign = sign - mean(sign);
    Nsamps = length(sign);


    T = 350e-6;            %               
    Fs = 1/T;             %       
    L = Nsamps;             % Length of signal
    t = (0:L-1)*T;        % Time vector


    f = Fs*(0:(L/2))/L;
    Y = fft(sign);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    P1_smoothed=smoothdata(P1);



    df = Fs/Nsamps; % frequency increment
    Y_squared = (Y/Fs).*conj(Y/Fs);  % Fs is used to normalize the FFT amplitudes
    energy = 2*sum(Y_squared( f>=10 & f<=500,: ))*df


    plot(f,P1_smoothed) 
    xlim([0 500]);
    ylim([0, 7e-3]);
    set_title = sprintf('%d Spectrum, enegy : %d', choice, energy);
    title(set_title)
    xlabel('f (Hz)')
    ylabel('|P1(f)|')

end

