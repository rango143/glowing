close all;
clc;
clear all;

% RS(188,204)
N = 255;
K = 239;
Short_Len = 51;

shortened_N = N - Short_Len;%N = 204
shortened_K = K - Short_Len;%K = 188

RS_rate = shortened_K/(N - Short_Len);
gpoly = rsgenpoly(N,K,[],0); %get RS polynomial
RS_encoder = comm.RSEncoder(N,K,gpoly,shortened_K);%RS encoder
RS_deccoder = comm.RSDecoder(N,K,gpoly,shortened_K);%RS decoder

% Interleaver parameters
rows = 12;
slope = 17; % Interleaver parameters
Delay = rows*(rows-1)*slope;  % Delay of interleaver/deinterleaver pair

RS_Int = comm.ConvolutionalInterleaver('NumRegisters',rows,'RegisterLengthStep', slope);
RS_Deint = comm.ConvolutionalDeinterleaver('NumRegisters',rows,'RegisterLengthStep', slope);

%Convolutional encoder
Conv_Enc = comm.ConvolutionalEncoder(poly2trellis(7,[133 171]));%Question 1 baseband simulator
Conv_Enc.PuncturePatternSource = 'Property';

%QPSK modulator
Q_mod = comm.QPSKModulator('BitInput',1);%Option to output data as bits, specified as a logical 0 (false) or 1 (true).
Q_demod = comm.QPSKDemodulator('BitOutput',1);

%AWGN channel
Q_channel = comm.AWGNChannel('NoiseMethod','Signal to noise ratio (Eb/No)','SignalPower',1,'SamplesPerSymbol',1,'BitsPerSymbol',1);
Error_Cal = comm.ErrorRate;

%Vitebi decoder
Q_VitDec = comm.ViterbiDecoder(poly2trellis(7,[133 171]),'InputFormat','Hard');% decoder is trellis representation, decoder takes hard input bits
Q_VitDec.PuncturePatternSource  = 'Property';%consider punctured pattern
Q_VitDec.TracebackDepth = 96;


%select puncture pattern
for puncturing = {'1/2','2/3','5/6'}
    i = 1;
    rate = str2num(puncturing{i});
    if rate == 1/2
        Conv_Enc.PuncturePattern = [1;1];%code rate of 1/2
    elseif rate == 2/3
         Conv_Enc.PuncturePattern =  [1;0;1;1]; %code rate of 2/3
    else
        Conv_Enc.PuncturePattern =  [1;0;1;0;1;1;1;0;1;0]; %code rate of 5/6
    end
    %modulator and demodulator
  
    M = 4;  %QPSK
    L = 11;
    frame_len  = 9;
    target_error = 1e3;
    max_Num_Trans= 1e5;
    Conv_Rate = 0.75;
    Coding_Rate = RS_rate*Conv_Rate; %RS code rate and convolutional  affect SNR
    BER_RS = [];%store all results

    EbNo_Input = 0:.2:10;%Input range
    EbN0_Output = EbNo_Input + 10*log10(log2(M)*rate*RS_rate); 
    Q_VitDec.PuncturePattern = Conv_Enc.PuncturePattern;
    BER_Vec = zeros(3,length(EbN0_Output));
    
    %simulation loop
    for n = 1:length(EbN0_Output)
        Q_channel.EbNo = EbN0_Output(n);
        fprintf('The simulation code rate is %s ,Eb/No is %g\n',puncturing{i},EbNo_Input(n));
        reset(Error_Cal);
        reset(Conv_Enc);
        reset(Q_VitDec);
     
        while(BER_Vec(2,n) < target_error)&&(BER_Vec(3,n)<max_Num_Trans)
            
            data = randi([0 255],188*frame_len,1);% get a TX data matrix from 0-255 
            
            zero = zeros(11*shortened_K,1);
            matrix = [data;zero];%combine two matrices
            % RS encoded
            RS_encode = step(RS_encoder,matrix);%RS encoder

            % interleaver
            Conv_Int_data = step(RS_Int,RS_encode);%to interleaver encoder
            all_RS = de2bi(Conv_Int_data)';%convert to binary bits  
            all_RS_data = all_RS(:);%get all values

            % convolutional modulator
            Conv_enc_data = step(Conv_Enc, all_RS_data);%convolutional encoder
            mod_data = step(Q_mod,Conv_enc_data);%modulate data           
            channel_output = step(Q_channel,mod_data); %through AWGN
            %demodulator
            demod_data = step(Q_demod, channel_output);% demodulate data           
            de_data = step(Q_VitDec, demod_data);
            de_data_Int = [de_data((Q_VitDec.TracebackDepth+1):end,1); zeros(Q_VitDec.TracebackDepth,1)];        

            de_data_shaped = reshape(de_data_Int,8,204*20)';%reshape data     
            binary = bi2de(de_data_shaped);%binary to decimal           
            RS_Deint_data = step(RS_Deint,binary);           
            RS_Dec_data = step(RS_deccoder,RS_Deint_data);        
            RS = RS_Dec_data(2069:end,1);%same input
            BER_Vec(:,n) = step(Error_Cal,data,RS);
        end
    end
    coded_Q = BER_Vec(1,:);
    uncoded_Q = berawgn(EbNo_Input,'psk',4,'diff');%Returns the value of an unencoded psk that was consistently detected on the AWGN channel.
    release(Conv_Enc);
    release(Q_VitDec);
    
     if rate == 1/2
       semilogy(EbNo_Input,uncoded_Q,'r-x',EbNo_Input,coded_Q,'g-o');%plot uncoded QPSK
       hold on;
    end
    if rate == 2/3
       Line2=semilogy(EbNo_Input,coded_Q,'b-o');
       hold on;

    end
    if rate == 5/6
       Line3 = semilogy(EbNo_Input,coded_Q,'k-o');
     
       hold on;
    end
    i = i + 1;
    grid on;
    xlabel('Eb/No ratio [dB]');
    title('BER vs Eb/No for QPSK with RS');
    ylabel('BER');
    
    BER_RS = [BER_RS;coded_Q];
    save('BER_QPSK_RS', 'EbNo_Input', 'BER_RS','uncoded_Q')%save result
end
legend('Uncoded QPSK','Rate = 1/2','Rate = 2/3','Rate = 5/6');

