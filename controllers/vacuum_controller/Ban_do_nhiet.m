% Ban_do_nhiet.m % Vẽ biểu đồ nhiệt từ dữ liệu heatmap_data.csv(lưới 15x15) %
    Chạy sau khi Webots hoàn thành simulation.

    % 1. Đọc dữ liệu CSV data = csvread('heatmap_data.csv');

% 2. Vẽ biểu đồ nhiệt figure;
imagesc(data);

% 3. Lật trục Y để khớp tọa độ thực tế(hàng 0 ở dưới)
        set(gca, 'YDir', 'normal');

% 4. Đổi bảng màu colormap(hot);
colorbar;

% 5. Nhãn trục xlabel('Cột lưới (0-14)');
ylabel('Hàng lưới (0-14)');
title('Bản đồ nhiệt - Số lần robot đi qua mỗi ô lưới');

% 6. Hiển thị giá trị trên mỗi ô[rows, cols] = size(data);
for i = 1:rows
    for j = 1:cols  
        if data(i,j) > 0
            text(j, i, num2str(data(i,j)), ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 7, 'Color', 'white');
end end end
