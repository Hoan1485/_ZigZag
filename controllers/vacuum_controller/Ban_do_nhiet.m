% Ban_do_nhiet.m
% Ve bieu do nhiet tu du lieu heatmap_data.csv (luoi 15x15)
% Chay sau khi Webots hoan thanh simulation.

% 1. Doc du lieu CSV
data = csvread('heatmap_data.csv');

% 2. Ve bieu do nhiet
figure;
imagesc(data);

% 3. Lat truc Y de khop toa do thuc te (hang 0 o duoi)
set(gca, 'YDir', 'normal');

% 4. Doi bang mau
colormap(hot);
colorbar;

% 5. Nhan truc
xlabel('Cot luoi (0-14)');
ylabel('Hang luoi (0-14)');
title('Ban do nhiet - So lan robot di qua moi o luoi');

% 6. Hien thi gia tri tren moi o
[rows, cols] = size(data);
for i = 1:rows
    for j = 1:cols
        if data(i,j) > 0
            text(j, i, num2str(data(i,j)), ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 7, 'Color', 'white');
        end
    end
end
