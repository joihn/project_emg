%%

testFile = hri_load_logfile("C:\Users\maxim\Google Drive\Epfl\MA4\haptic\project_emg\end_test5.csv")


time = testFile.timestamp__s_ 
time = time - time(1)
index = time > 1.8
time=time(index)
time = time - time(1)
%% raw
plot(time, testFile.amplifier_value(index))
title("Raw EMG signal")
xlabel("Time [s]")
ylabel("Magnitue [V]")
xlim([0, 9])
%% bad pass and notch
plot(time, testFile.filtered_value(index))
title("Signal after pass band in [10,200] Hz and notch filter at 50Hz")
xlabel("Time [s]")
ylabel("Magnitue [V]")
xlim([0, 9])
%% badn pass and notch spectrum
spectrum_plotter(testFile.filtered_value/4, "Spectrum after pass band and notch filter")
%%
hold on
box on
plot(time, testFile.Energy_FFT(index), 'DisplayName', 'Original RMS')
plot(time, testFile.muscleOutput(index), 'DisplayName', 'Smoothed RMS')
title("RMS value computed on a sliding windows of 0.35s")
legend('Location','northwest')
xlabel("Time [s]")
ylabel("Magnitue")
xlim([0, 9])

%%

function energy = spectrum_plotter(value, titla)
    plot_energy =1
    
    % sign = sign - mean(sign);
    Nsamps = length(value);


    T = 1e-3;            %               
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
    energy = 2*sum(Y_squared( f>=10 & f<=500,: ))*df - 2*sum(Y_squared( f>=45 & f<=55,: ))*df;

    if plot_energy == 1
%         plot(f,P2_smoothed)
        
        plot(f,P1_smoothed)
        xlim([0 500]);
        ylim([0, 2e-3]);
      
        title(titla)
        xlabel('f [Hz]')
        ylabel('Magnitude')
    end

end
    