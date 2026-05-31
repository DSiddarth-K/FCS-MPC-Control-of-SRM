%% TABLE SETUP
close all; clc;

N_angles   = length(elecAngleArray);
N_fluxlink = N_angles;

LUT_currentMatrix = zeros(N_angles,N_fluxlink);
targetArray = linspace(min(fluxLinkageMatrix(:)),max(fluxLinkageMatrix(:)),N_fluxlink);
current_max = max(currentArray);

for i_angle = 1:N_angles
    fluxlink_row = fluxLinkageMatrix(i_angle,:);
    
    for i_fluxlink = 1:N_fluxlink
        TARGET = targetArray(i_fluxlink);
        
        if TARGET > max(fluxlink_row)
            LUT_currentMatrix(i_angle,i_fluxlink) = current_max;
        else
            LUT_currentMatrix(i_angle,i_fluxlink) = ...
                interp1(fluxlink_row, currentArray, TARGET);
        end
    end
end

% Plot flux linkage look-up table
figure(1)
mesh(currentArray,elecAngleArray,fluxLinkageMatrix);
xlim([min(currentArray), max(currentArray)]);
ylim([min(elecAngleArray), max(elecAngleArray)]);
xlabel('Current [A]');
ylabel('Electrical Angle [deg]');
zlabel('Flux Linkage [Wb]');

% Plot current look-up table
figure(2)
mesh(targetArray,elecAngleArray,LUT_currentMatrix);
xlim([min(targetArray), max(targetArray)]);
ylim([min(elecAngleArray), max(elecAngleArray)]);
xlabel('Flux Linkage [Wb]');
ylabel('Electrical Angle [deg]');
zlabel('Current [A]');
%%