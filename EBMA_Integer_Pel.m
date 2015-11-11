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

[h1 w1] = size(anchor_frame);

N=16; % block size
R=16; % search range

anchor_frame_p = padarray(anchor_frame,[16 16]);
figure(3);
imshow(anchor_frame_p, []);
title('Anchor Frame after Padding');
target_frame_p = padarray(target_frame, [16 16]);
figure(4);
imshow(target_frame_p, []);
title('Target Frame after Padding');

%Integer-Pel EBMA

mvx=0;
mvy=0;

for i=17:N:h1+N
    for j=17:N:w1+N  %for every block in the anchor frame       
        MAD_min=256*N*N;
        
        for k=-R:1:R
            for l=-R:1:R  %for every search candidate
                MAD=sum(sum(abs(anchor_frame_p(i:i+N-1,j:j+N-1)-target_frame_p(i+k:i+k+N-1,j+l:j+l+N-1))));
                % calculate MAD for this candidate
                if MAD<MAD_min
                    MAD_min=MAD; 
                    dy=k;
                    dx=l;
                end
            end
        end
        predicted_frame((i-N):(i-N)+N-1,(j-N):(j-N)+N-1)= target_frame_p(i+dy:i+dy+N-1,j+dx:j+dx+N-1); 
        %put the best matching block in the predicted image
        iblk=floor((i)/(N)); 
        jblk=floor((j)/(N)); %block index
        mvx(iblk,jblk)=dx; 
        mvy(iblk,jblk)=dy; %record the estimated MV
    end
end

%predicted_frame = uint8(predicted_frame);
figure(5);
imshow(uint8(predicted_frame), [0 255]);
title('Predicted Image');

figure(6);
quiver(mvx,mvy);
title('Estimated Motion Field');

error_frame = zeros(288, 352);
for a = 1:h1
    for b = 1:w1
       error_frame(a,b)=anchor_frame(a,b)-predicted_frame(a,b);
    end
end
figure(7);
imshow(error_frame, [0 255]);
title('Prediction Error Image');

squared_error = error_frame.^2;
%calulating PSNR
sum = 0;
for x = 1:h1
    for y = 1:w1
        sum = sum + squared_error(x,y);
    end
end

var = sum/(288*352);

PSNR = 10*log10((255*255)/var);
display(PSNR);




% Problem 3

mvx1 = imresize(mvx, 16, 'bilinear');
mvy1 = imresize(mvy, 16, 'bilinear');

figure(8);
quiver(mvx1, mvy1);
title('Pixel Based Motion Field obtained using Interpolation');


for i=17:h1+N
    for j=17:w1+N  %for every block in the anchor frame       
        predicted_frame_1((i-N),(j-N))= target_frame_p(i+floor(mvy1((i-N),(j-N))),j+floor(mvx1((i-N),(j-N)))); 
    end
end

figure(9);
imshow(predicted_frame_1, []);
title('Predicted Image from Pixel Based Motion Field');

error_frame_1 = zeros(288, 352);
for a = 1:h1
    for b = 1:w1
       error_frame_1(a,b)=anchor_frame(a,b)-predicted_frame_1(a,b);
    end
end
figure(10);
imshow(error_frame_1, [0 255]);
title('Prediction Error Image');

squared_error_1 = error_frame_1.^2;
%calulating PSNR
sum2 = 0;
for x = 1:h1
    for y = 1:w1
        sum2 = sum2 + squared_error_1(x,y);
    end
end

var1 = sum2/(288*352);

PSNR1 = 10*log10((255*255)/var1);
display(PSNR1);
        


