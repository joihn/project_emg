
%%
variables = hri_load_logfile("../media/biceps_bas/force_forte_coude_int.csv");

emg_signal = variables.emg_signal;
% emg_signal = emg_signal - mean(emg_signal);
position = variables.paddle_pos__deg_;
%%
threshold = 0;
bin_array= position > threshold;

figure
plot(emg_signal)
hold on;
plot(bin_array/6)

%% Moving standard deviation
moving_std = movstd(emg_signal,50);
moving_mean = sqrt(movmean(emg_signal .^ 2, 50));
figure
plot(emg_signal)
hold on;
plot(bin_array/6)
hold on;
plot(moving_std)
hold on;
plot(moving_mean)

%% Moving energy (just to see the result)
choice = -1;
energy_values = [];
energy_time_values = [];
window_size = 1000;
nb_step = length(emg_signal) - window_size;
for i = 1 : nb_step -1
    value = emg_signal(i : (i+window_size));
    energy = energy_freqeunccy_domain(choice, value, 0);
    energy_values = [energy_values, energy];
%     enegy_time = energy_time_domain(value);
%     energy_time_values = [energy_time_values, enegy_time];
end

%% plot energy across windows
% figure
% plot(moving_std/20)
% hold on
% plot(smoothdata(moving_std)/20)
% hold on
% plot(energy_values)
% legend("moving std", "moving std smoothed", "energy")
% 
figure
plot(smoothdata(moving_std)/20)
hold on
plot(energy_values)
hold on
plot(energy_time_values)
legend("moving std smoothed", "energy", "energy time domain")
title("Energy vs moving std on weak force")


%%
on_whole= emg_signal(bin_array);
off_whole= emg_signal(~bin_array);
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

% close all
figure
for choice = 0:1
    if choice == 0
    value = off_whole;
    else
        hold on;
        value = on_whole;
    end
    energy = energy_freqeunccy_domain(choice, value, 1);
end


%%
function energy = energy_freqeunccy_domain(choice, value, plot_energy)
    
    
    % sign = sign - mean(sign);
    Nsamps = length(value);


    T = 350e-6;            %               
    Fs = 1/T;             %       
    L = Nsamps;             % Length of signal


    f = Fs*(0:(L/2))/L; %pourquoi la taille de f est deux fois plus petite ?
%     f = Fs*(0:L)/L; %pourquoi la taille de f est deux fois plus petite ?
    Y = fft(value);
    P2 = abs(Y/L);
    P1 = P2(1:fix(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    P1_smoothed=smoothdata(P1);
    P2_smoothed=smoothdata(P2);



    df = Fs/Nsamps; % frequency increment
    Y_squared = (Y/Fs).*conj(Y/Fs);  % Fs is used to normalize the FFT amplitudes
    energy = 2*sum(Y_squared( f>=10 & f<=500,: ))*df - 2*sum(Y_squared( f>=130 & f<=160,: ))*df;

    if plot_energy == 1
%         plot(f,P2_smoothed)
        plot(f,P1_smoothed)
        xlim([0 500]);
        ylim([0, 7e-3]);
        set_title = sprintf('%d Spectrum, enegy : %d', choice, energy);
        title(set_title)
        xlabel('f (Hz)')
        ylabel('|P2(f)|')
    end

end


function energy = energy_time_domain(value)

    T = 350e-6;                          
    Fs = 1/T; 
    
    X_filtered = bandpass(value, [10, 500], Fs);
    X_filtered = bandstop(X_filtered, [130, 160], Fs);
    energy = sum(X_filtered.^2)/Fs;

end



