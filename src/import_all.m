function  [all_data] = import_all()


    %% YOUR BASE FOLDER
    allSubFolders = genpath('../media');

    % Parse into a cell array.
    remain = allSubFolders;
    listOfFolderNames = {};
    while true
        [singleSubFolder, remain] = strtok(remain, ';');
        if isempty(singleSubFolder)
            break;
        end
        listOfFolderNames = [listOfFolderNames singleSubFolder];
    end
    numberOfFolders = length(listOfFolderNames)

    all_data = []
    for k = 1 : numberOfFolders
        % Get this folder and print it out.
        thisFolder = listOfFolderNames{k};
        fprintf('Processing folder %s\n', thisFolder);

        % Get the csv files
        filePattern = sprintf('%s/*.csv', thisFolder);
        baseFileNames = dir(filePattern);

        numberOfImageFiles = length(baseFileNames);
        if numberOfImageFiles >= 1
            % Go through all those files.
            for f = 1 : numberOfImageFiles
                fullFileName = fullfile(thisFolder, baseFileNames(f).name);
                fprintf('     Processing file %s\n', fullFileName);

                variables = hri_load_logfile(fullFileName);
                emg_signal = variables.emg_signal;
                emg_signal = emg_signal - mean(emg_signal);
                position = variables.paddle_pos__deg_;
                threshold = 0;
                bin_array= position > threshold;

                %_____________________TODO_______________________________ :
    %             put the variables (emg_signal, bin_array) in some data
    %             structure with the label baseFileNames(f).name
    %               maybe we can use table, or struct, i'm not sure

                data.emg_signal = emg_signal
                data.bin_array = bin_array
                data.name = baseFileNames(f).name
                all_data = [all_data ; data]
            end
        else
            fprintf('     Folder %s has no files in it.\n', thisFolder);
        end
    end
end

