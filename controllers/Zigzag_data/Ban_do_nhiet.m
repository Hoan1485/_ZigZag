% 1. Ð?c d? li?u t? file CSV v?a upload
data = csvread('heatmap_data.csv');

% 2. C?t ph?n vi?n s? 0 (Focus vào tâm ph?ng 5x5m)
room_data = data(35:65, 35:65);

% 3. V? bi?u ð? nhi?t 
figure;
imagesc(room_data);

% 4. L?t tr?c Y l?i cho ðúng h? t?a ð? th?c t?
set(gca, 'YDir', 'normal');

% 5. Ð?i tông màu sang cam/ð?/vàng
colormap(hot);
colorbar; 

% 6. Thêm tiêu ð? (Không d?u ð? tránh l?i font trên b?n 2016a)
title('Bieu do trum lap quy dao (Path Overlap)');