data = csvread('heatmap_data.csv');

% 2. Vẽ biểu đồ nhiệt figure;
imagesc(data);

% 3. Lật trục Y để khớp tọa độ thực tế(hàng 0 ở dưới)
        set(gca, 'YDir', 'normal');
axis equal;
axis tight;

colormap(hot);
colorbar;
xlabel('Cột lưới');
ylabel('Hàng lưới');
title('Biểu đồ trùng lặp quỹ đạo');
