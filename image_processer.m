% Capture simulation
img = imread('earth_sim.jpg');
img_resized = imresize(img, [480 640]);
imwrite(img_resized, 'payload_image.jpg');