clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
R_max = 200;
d_res = 1;
v_max = 100;


%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
p0 = 50;
v0 = -20;
c = 3e8;


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c/(2*d_res);
Tchirp = 5.5*(R_max*2/c);
slope = B/Tchirp;

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = p0 + v0*t(i);
    
    td(i) = 2*r_t(i)/c;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    t_slope = (t(i)-floor(t(i)/Tchirp)*Tchirp);
    Tx(i) = cos(2*pi*(fc*t(i)+slope*t_slope*t(i)));
    t_slope = ((t(i)-td(i))-floor((t(i)-td(i))/Tchirp)*Tchirp);
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+slope*t_slope*(t(i)-td(i))));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i)*Rx(i);
    
end

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
% Mix = reshape(Mix,1,[]);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
FFT_Mix = fft(Mix)/(Nr*Nd);

 % *%TODO* :
% Take the absolute value of FFT output
FFT_Mix = abs(FFT_Mix);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
FFT_Mix = FFT_Mix(1:(Nr*Nd)/2+1);
FFT_Mix(2:end-1) = 2*FFT_Mix(2:end-1);


%plotting the range
figure('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
f = ((Nr*Nd)/(Nd*Tchirp))*(1:((Nr*Nd)/2+1))/(Nr*Nd);
r = f*c/(4*slope);

nexttile
plot(r,FFT_Mix) 
title('Range from First FFT')
xlabel('range (m)')
ylabel('f(x)')

axis([0 200 0 1]);



Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift(sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = (Nr/Tchirp)*(-(Nr/4)+1:(Nr/4))/Nr*c/(4*slope);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

signal_cfar = zeros(1,Nr/2*Nd);
signal_cfar = reshape(signal_cfar,[Nr/2,Nd]);

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 2;
Td = 2;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 1;
Gd = 1;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 6;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.
for i = 1:(Nr/2-(2*Tr+2*Gr+1))     
    for j = 1:(Nd-(2*Td+2*Gd+1))
        % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
        % CFAR

        % TODO: Determine the noise threshold by measuring it within
        % the training cells
        noise_level = sum(db2pow(RDM(i:i+2*Tr+2*Gr+1,j:j+2*Td+2*Gd+1)),'all') - sum(db2pow(RDM(i+Tr:i+Tr+2*Gr+1,j+Td:j+Td+2*Gd+1)),'all');
        % TODO: scale the noise_level by appropriate offset value and take
        % average over T training cells
        threshold = (noise_level/((2*Tr+2*Gr+1)*(2*Td+2*Gd+1)-(2*Gr+1)*(2*Gd+1)))*offset;
        % Add threshold value to the threshold_cfar vector
        threshold_cfar = pow2db(threshold);
        % TODO: Measure the signal within the CUT
        signal = RDM(i+Tr+Gr,j+Td+Gd);
        % add signal value to the signal_cfar vector
        if signal>threshold_cfar
            signal_cfar(i+Tr+Gr,j+Td+Gd) = 1;
        end
    end
end

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,signal_cfar);
colorbar;
 