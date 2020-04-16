# sfnd-radar-target-generation-detection
This is the final project of lesson 3 "Radar Sensor" in "Sensor Fusion" Nanodgree in Udacity. 
This course implementations are done in Matlab.

## Project Rubrics:

##### 1) Using the given system requirements, design a FMCW waveform. Find its Bandwidth (B), chirp time (Tchirp) and slope of the chirp

```matlab
% 1. Calculate the Bandwidth (B)
B = c / 2 * dres;

% 2. Chirp Time (Tchirp)
Tchirp = 5.5 * 2 * Rmax / c;

% 3. Slope (slope) of the FMCW
fmcw_slope = B / Tchirp;
```

##### 2) Simulate Target movement and calculate the beat or mixed signal for every timestamp.

```matlab
%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 
for i=1:length(t)         
    
    %For each time stamp update the Range of the Target for constant velocity.
    td(i) = target_range + (t(i) * target_velocity);
    r_t(i)= 2 * td(i) / c;
    
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2 * pi * (fc*t(i) + fmcw_slope * t(i)^2 / 2));
    Rx(i) = cos(2 * pi * (fc*(t(i) - r_t(i)) + fmcw_slope * (t(i) - r_t(i))^2 / 2));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) * Rx(i);
end
```

##### 3) Implement the Range FFT on the Beat or Mixed Signal and plot the result.

```matlab
%% RANGE MEASUREMENT

%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.

fb = reshape(Mix, [Nr, Nd]);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.

fft_signal_2 = fft(fb) / Nr;

% Take the absolute value of FFT output

fft_signal_2 = abs(fft_signal_2);

% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.

fft_signal_1 = fft_signal_2(1 : Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

% plot FFT output 

x = 0 : length(fft_signal_1) - 1;

plot(x, fft_signal_1);
axis ([0 200 0 1]);
```

##### 4) Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.

1. Implementation steps for the 2D CFAR process

```matlab
% slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
cfar_signals = zeros(size(RDM));
[range_cells_size, doppler_cells_size] = size(RDM);
for i = (Tr + Gr + 1) : (range_cells_size - (Tr + Gr))
    for j = (Td + Gd + 1) : (doppler_cells_size - (Td + Gd))
        
        %Create a vector to store noise_level for each iteration on training cells
        noise_level = zeros(1,1);
        
        % For every iteration sum the signal level within all the training
        % cells. 
    
        for p = i - (Tr + Gr) : i + Tr + Gr
            for q = j - (Td + Gd) : j + Td + Gd
                
                if (abs(i-p)>Gr || abs(j-q)>Gd)
                    % To sum convert the value from logarithmic to linear using
                    % db2pow function.

                    noise_level = noise_level + db2pow(RDM(p,q));
                end
                
            end
        end
        
        % Average the summed values for all of the training cells used.
        % After averaging convert it back to logarithimic using pow2db.
        
        threshold = pow2db(noise_level / train_cells_num);
    
        %Further add the offset to it to determine the threshold.
        
        threshold = threshold * offset;
        
        % Next, compare the signal under CUT with this threshold.
        % If the CUT level > threshold assign it a value of 1,
        % else equate it to 0.
        
        CUT = RDM(i,j);
        if CUT > threshold
            cfar_signals(i,j) = 1;
        end
    end
end
```

2. Selection of Training, Guard cells and offset

```matlab
%Select the number of Training Cells in both the dimensions.

Tr = 10;
Td = 8;

%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation

Gr = 4;
Gd = 4;

% offset the threshold by SNR value in dB

offset = 1.5;

grid_size = 2*(Td+Gd+1)*2*(Tr+Gr+1);
train_cells_num = grid_size - (2*Gr+1)*(2*Gd+1);
```

3. Steps taken to suppress the non-thresholded cells at the edges.

Creation of range doppler new map signal with the same size as original one and initialzed with zeros.
```matlab
cfar_signals = zeros(size(RDM));
```

Then when checking the CUT against the calculated threshold in CFAR logic, if the CUT is more than the threshold then the current element in the new map signal is set to 1.

```matlab
CUT = RDM(i,j);
if CUT > threshold
    cfar_signals(i,j) = 1;
end
```

Which means the the cells at the edge are already initialized to zeros.