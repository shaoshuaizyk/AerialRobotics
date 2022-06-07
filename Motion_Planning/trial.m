
alphaNum = 0.1;
% figure('color','w')
figure()
hold on

for ii = 1:3

    % Vertex coordinates
    p1=[0 0 0] + rand(1, 3);
    p2=[2 0 0] + rand(1, 3);
    p3=[1 sqrt(3/2) sqrt(3/2)] + rand(1, 3);
    % Plot trianle using 'patch'

    h = patch('Faces',1:3,'Vertices',[p1;p2;p3]);
    set(h, 'FaceColor', 'b', 'EdgeColor', 'b', 'LineWidth', 1,...
            'FaceAlpha', alphaNum, 'EdgeAlpha', alphaNum);
    


end

hold off
view([30, 30])
xlabel('x','FontSize',20)
ylabel('y','FontSize',20)
zlabel('z','FontSize',20)