% 1. Doc du lieu tu file CSV vua upload
data = csvread('heatmap_data.csv');

% 2. Su dung toan bo ban do 15x15 (Khong can cat vien nua)
room_data = data;

% 3. Ve bieu do nhiet 
figure;
imagesc(room_data);

% 4. Lat truc Y lai cho dung he toa do thuc te
set(gca, 'YDir', 'normal');

% 5. Doi tong mau sang cam/do/vang de de nhin
colormap(hot);
colorbar; 

% 6. Them tieu de (Khong dau de tranh loi font tren ban 2016a)
title('Bieu do trum lap quy dao (Path Overlap)');
