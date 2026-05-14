data = csvread('heatmap_data.csv');
room_data = data; 
figure(1)
imagesc(room_data);
caxis([0 5]);
set(gca, 'YDir', 'normal');
colormap(hot);
colorbar; 
title('Bieu ðo trung lap quy ðao');