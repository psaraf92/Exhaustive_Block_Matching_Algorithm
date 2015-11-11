clc;
clear all;

H=352; % height of frame
W=288; % width of frame
f1=fread(fopen('foreman72.Y','r'),[H,W]); % Read .Y files
anchor_frame = imrotate(f1, -90);
f2=fread(fopen('foreman66.Y','r'),[H,W]);
target_frame = imrotate(f2, -90);
figure(1);
imshow(anchor_frame, []);
title('Anchor Frame');
figure(2);
imshow(target_frame, []);
title('Target Frame');

N=16; % block size
R=16; % search range

anchor_frame_hp_p = padarray(anchor_frame,[32 32]);
figure(3);
imshow(anchor_frame_hp_p, []);
title('Anchor Frame after Zero Padding');

% Half-Pel EBMA

[H1, W1] = size(target_frame);

target_frame_hp(1:2:H1*2-1,1:2:W1*2-1)=target_frame(1:H1,1:W1);
target_frame_hp(1:2:H1*2-1,2:2:W1*2-2)=(target_frame(1:H1,1:W1-1)+target_frame(1:H1,2:W1))/2;
target_frame_hp(2:2:H1*2-2,1:2:W1*2-1)=(target_frame(1:H1-1,1:W1)+target_frame(2:H1,1:W1))/2;
target_frame_hp(2:2:H1*2-2,2:2:W1*2-2)=(target_frame(1:H1-1,1:W1-1)+target_frame(1:H1-1,2:W1)+target_frame(2:H1,1:W1-1)+target_frame(2:H1,2:W1))/4;

target_frame_hp1 = padarray(target_frame_hp, [1 1], 'replicate', 'post');
figure(4);
imshow(target_frame_hp1, []);
title('Target Frame after Interpolation');

target_frame_hp_p = padarray(target_frame_hp1, [64 64]);

figure(5);
imshow(target_frame_hp_p, []);
title('Target Frame after Interpolation and Padding'); 

dx_hp=0;
dy_hp=0;

for s=33:N:H1+32
    for t=33:N:W1+32  %for every block in the anchor frame       
        MAD_min=256*N*N;
        for u=-R:0.5:R
            for v=-R:0.5:R  %for every search candidate
                MAD=sum(sum(abs(anchor_frame_hp_p(s:s+N-1,t:t+N-1)-target_frame_hp_p((s+u)*2-1:2:(s+u+N-1)*2-1,(t+v)*2-1:2:(t+v+N-1)*2-1))));
                % calculate MAD for this candidate
                if MAD<MAD_min
                    MAD_min=MAD;
                    dy_hp=u;
                    dx_hp=v;
                end;
            end;
        end;
        predicted_frame_hp((s-32):(s-32)+N-1,(t-32):(t-32)+N-1)= target_frame_hp_p((s+dy_hp)*2-1:2:(s+dy_hp+N-1)*2-1,(t+dx_hp)*2-1:2:(t+dx_hp+N-1)*2-1); 
        %put the best matching block in the predicted image
        iblk_hp= floor(s/N); 
        jblk_hp= floor(t/N); %block index
        mvx_hp(iblk_hp,jblk_hp)=dx_hp; 
        mvy_hp(iblk_hp,jblk_hp)=dy_hp; %record the estimated MV
    end;
end;

figure(6);
imshow(predicted_frame_hp, []);
title('Predicted Image');

figure(7);
quiver(mvx_hp, mvy_hp);
title('Estimated Motion Field');

error_frame_hp = zeros(288, 352);
for a1 = 1:H1
    for b1 = 1:W1
       error_frame_hp(a1,b1)=anchor_frame(a1,b1)-predicted_frame_hp(a1,b1);
    end
end
figure(8);
imshow(error_frame_hp, [0 255]);
title('Prediction Error Image');


squared_error_hp = error_frame_hp.^2;
%calulating PSNR
sum_hp = 0;
for x1 = 1:H1
    for y1 = 1:W1
        sum_hp = sum_hp + squared_error_hp(x1,y1);
    end
end

var_hp = sum_hp/(288*352);

PSNR_hp = 10*log10((255*255)/var_hp);
display(PSNR_hp);
