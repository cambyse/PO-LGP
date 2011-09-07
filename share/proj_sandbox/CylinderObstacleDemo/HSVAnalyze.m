load img.hsv%%vector of byte values

w = 1024;
h = size(img,1)/1024;

rgb = reshape(hsv2rgb(img/256),w,h,3);

subplot(2,2,1)
image(rgb);

for i = 1:3
   im = reshape(img(:,i),w,h);
    subplot(2,2,i+1)
    imagesc(im)   
end
