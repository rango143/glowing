
close all;
clc;

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
EbNo_Input = 0:0.5:10;% range 
frame_Len = 12000;
target_Error = 1e5;
max_Num_Trans = 1e7;
BER = [];



%select puncture pattern
for puncturing = {'1/2','2/3','5/6'}
    i = 1;
    rate = str2num(puncturing{i});
    if rate == 1/2
       Conv_Enc.PuncturePattern =[1;1];  
    elseif rate == 2/3
        Conv_Enc.PuncturePattern =[1;0;1;1];
    elseif rate == 5/6
        Conv_Enc.PuncturePattern =[1;0;1;0;1;1;1;0;1;0];
    end
    
    EbNo_Output = EbNo_Input +10*log10(rate*log2(4));%code Eb/No rate
    Q_VitDec.PuncturePattern = Conv_Enc.PuncturePattern;
    BER_Vec = zeros(3,length(EbNo_Output));%store all results
    for n = 1:length(EbNo_Output)
        reset(Error_Cal);
        reset(Conv_Enc);
        reset(Q_VitDec);
        
        Q_Channel.EbNo = EbNo_Output(n);
        
        fprintf('code rate is %s,Eb/No is %g\n',puncturing{i},EbNo_Input(n));

        
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
    
    banner = {'uncoded QPSK '};
    uncoded_Q = berawgn(EbNo_Input,'psk',4,'diff');%Returns the value of an unencoded psk that was consistently detected on the AWGN channel.
   

 
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
    title('BER vs Eb/No for QPSK');
    ylabel('BER');
    
    hold on;

    BER_AWGN = [BER;coded_Q];
    save('BER_AWGN_compare','EbNo_Input','BER_AWGN','uncoded_Q')%save results
end
legend('Uncoded QPSK','Rate=1/2','Rate=2/3','Rate = 5/6');