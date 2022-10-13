close all;
clc;
clear all;
%convolutional encoder
Conv_Enc = comm.ConvolutionalEncoder(poly2trellis(7,[133 171])); %matlab 2 baseband simulator
Conv_Enc.PuncturePatternSource = 'Property';%puncturing pattern used in DVB-S

%modulator and demodulator
Q_Mod = comm.QPSKModulator('BitInput',1);%Option to output data as bits, specified as a logical 0 (false) or 1 (true).
Q_Demod = comm.QPSKDemodulator('BitOutput',1);
Q_Channel = comm.AWGNChannel('NoiseMethod','Signal to noise ratio (Eb/No)','SignalPower',1,'SamplesPerSymbol',1,'BitsPerSymbol',1); %AWGN channel

%Viterbi decoder
Q_VitDec = comm.ViterbiDecoder(poly2trellis(7,[133 171]),'InputFormat','Hard');% decoder is trellis representation, decoder takes hard input bits
Q_VitDec.PuncturePatternSource  = 'Property'; %consider punctured pattern


Q_VitDec.TracebackDepth = 96;
Error_Cal = comm.ErrorRate('ReceiveDelay',Q_VitDec.TracebackDepth); %the traceback of viterbi decoder lead to a delay

%set parameters
EbNo_Input = 0:0.2:10;% range 

EbNo_Output = EbNo_Input +10*log10((1/2)*log2(4)); %code Eb/No rate
Conv_Enc.PuncturePattern = [1;1];%code rate of 1/2
Q_VitDec.PuncturePattern = Conv_Enc.PuncturePattern;

frame_Len = 12000;
target_Error = 1e5;
max_Num_Trans = 1e7;
BER = [];
BER_Vec = zeros(3,length(EbNo_Output));%store all results
%simulation loop 1/2
for n = 1:length(EbNo_Output)
    reset(Error_Cal);
    reset(Conv_Enc);
    reset(Q_VitDec);
    
    Q_Channel.EbNo = EbNo_Output(n);
    
    fprintf('code rate is 1/2, Eb/No is %g\n',EbNo_Input(n));
    
    while(BER_Vec(2,n) < target_Error)&&(BER_Vec(3,n)<max_Num_Trans)
        data = randi([0 1],frame_Len,1);%binary frames
        enc_data = step(Conv_Enc, data);%convolutional encoder
        mod_data = step(Q_Mod,enc_data);%modulate data
        channel_output = step(Q_Channel,mod_data);%through AWGN
        demod_data = step(Q_Demod, channel_output);%demodulate data
        de_data = step(Q_VitDec, (demod_data));%decode data
        
      
        BER_Vec(:,n) = step(Error_Cal, data, de_data);%compute errors
        
        
    end
end
coded_Q = BER_Vec(1,:);
release(Conv_Enc);
release(Q_VitDec);


uncoded_Q = berawgn(EbNo_Input,'psk',4,'diff');%Returns the value of an unencoded psk that was consistently detected on the AWGN channel.
BER = [BER;coded_Q];


figure;
semilogy(EbNo_Input,coded_Q,'b-o',EbNo_Input,uncoded_Q,'r-x');%plot semilogy figure
legend('coded QPSK','uncoded QPSK','Location','northeast')
grid on;

xlabel('Eb/No ratio [dB]');
title('BER vs Eb/No for QPSK at 1/2');
ylabel('BER');

save('QPSK1.mat','EbNo_Input','BER','uncoded_Q');%save results