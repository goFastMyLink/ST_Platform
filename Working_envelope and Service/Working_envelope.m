tic
i = 0;
for a1 = 0:18:360
    for a2 = 0:18:360
        for a3 = 0:18:360
            i=i+1;
            a = [a1 a2 a3];
            [VZ, EA] = PZK_fun(a);
            x(i) = VZ(1);
            y(i) = VZ(2);
            z(i) = VZ(3);
        end
    end
end
% plot3(x, y, z, '--');
%%
scatter3(x,y,z,'MarkerEdgeColor','k','MarkerFaceColor',[.0 .75 .75]);
view(-30,10)
toc         