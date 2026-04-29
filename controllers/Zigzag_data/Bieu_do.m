% 1. Khai báo tr?c Y (Ph?n trãm hoàn thành t? 0% ð?n 100%)
y_percentage = 0:10:100;

% 2. Khai báo tr?c X (Th?i gian hoàn thành tính b?ng giây)
% NH?P S? LI?U T? CONSOLE C?A B?N VÀO ÐÂY:
time_random = [0, 18.4, 36.8, 59.3, 82.5, 116.3, 158.9, 213.4, 276.9, 411.1, 1578.2];
time_zigzag = [0, 20.1, 56.7, 112.0, 150.0, 171.6, 219.6, 261.8, 301.7, 355.6, 384.9];
time_spiral = [0, 24.1, 43.6, 62.4, 79.5, 98.1, 115.1, 132.7, 148.8, 176.6, 197.8];
time_snake  = [0, 17.5, 34.7, 54.8, 72.0, 92.2, 109.4, 129.5, 146.7, 166.8, 184.6];

% 3. Kh?i t?o Figure và v? 4 ðý?ng ð? th?
figure;
plot(time_random, y_percentage, '-b', 'LineWidth', 2); hold on; % Random (Xanh dýõng)
plot(time_zigzag, y_percentage, '-r', 'LineWidth', 2);          % Zigzag (Ð?)
plot(time_spiral, y_percentage, '-y', 'LineWidth', 2);          % Spiral (Vàng)
plot(time_snake,  y_percentage, '-g', 'LineWidth', 2);          % Snake (Xanh lá)

% 4. Tùy ch?nh thông s?, nh?n dán cho bi?u ð?
title('Ty le dien tich don dep theo thoi gian (Coverage over Time)');
xlabel('Thoi gian (giay)');
ylabel('Phan tram don dep (%)');

% 5. Hi?n th? chú thích (Legend) ? góc dý?i cùng bên ph?i ð? không che m?t ð? th?
legend('Random', 'Zigzag', 'Spiral', 'Snake', 'Location', 'SouthEast');

% 6. B?t lý?i ð? d? ð?i chi?u s? li?u
grid on;