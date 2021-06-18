%%
all_data= import_all()

%%
shape = size(all_data);
all_rest = [];
all_activate = [];
for n = (1: shape(1))
    if contains(all_data(n).name, "force")
        bool2 = all_data(n).bin_array;
        bool2 = double(bool2);
        value = 0.5;
        %rise
        locations = (bool2 >= value);
        diff_locations = [false ; diff(locations)];
        index_rise = (diff_locations ==1);

        %fall
        locations = (bool2 <= value);
        diff_locations = [false ; diff(locations)];
        index_fall = (diff_locations == 1);

        margin = 0.4/(1e-3);

        temp = find(index_rise)';
        for i = temp
            bool2(i-margin:i+margin)=0.5;

        end
        temp = find(index_fall)';
        for i = temp
            bool2(i-margin:i+margin)=0.5;
        end
        
        all_rest = [all_rest; all_data(n).emg_signal(bool2==0)];
        all_activate =[all_activate ; all_data(n).emg_signal(bool2==1)];
        
    
    end
    
end
%% raw signal 
energy_frequency_domain(0, all_rest, 1)
figure()
energy_frequency_domain(1, all_activate, 1)

%% filtering 
% low_filt = fir1(48,[10*(2*pi) 45/(2*pi)]);
% high_filt = fir1(48,[55/(2*pi) 200/(2*pi)]);

sampl_freq = 1/(350e-6);
high_filt = fir1(48,[10/sampl_freq*2*pi 200/sampl_freq*2*pi]);




%%
function energy = energy_frequency_domain(choice, value, plot_energy)
    
    
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
        if choice==0
            set_title = sprintf('Power spectrum at rest');
        else
            set_title = sprintf('Power spectrum during muscle activation');
        end
            title(set_title)
        xlabel('f [Hz]')
        ylabel('Magnitude')
    end

end
    


