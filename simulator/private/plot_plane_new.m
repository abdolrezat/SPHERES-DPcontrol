function plot_plane_new(~,x,y,theta2q2a,n_sampling,a,sep,col_g,col_gh,head_LineWidth,tick_spacing_n)
% sampling points:
% method 1:
i_sampling = round(linspace(1,length(x), n_sampling));
% method 2:
i_sampling = 1;
x0 = x(1);
%p0 = x(1)^2 + y(1)^2;
for counter = 1:length(x)
    if(abs(x(counter) - x0) > sep)
        %                  if(x(counter)^2 + y(counter)^2 - p0 > sep)
        i_sampling(end+1) = counter;
        x0 = x(counter);
        %                      p0 = x(counter)^2 + y(counter)^2;
        
    end
end

[b,ixorigin] = min(x.^2 + y.^2);
i_sampling(end) = ixorigin;
%              if(i_sampling(end) ~= counter)
%                  i_sampling(end + 1) = counter;
%              end

x_samples = x(i_sampling);
y_samples = y(i_sampling);
t_samples = theta2q2a(i_sampling);
%              t_samples = theta2_c(i_sampling);



Px_a = [a,a,-a,-a];
Py_a = [a,-a,-a,a];
figure('color', 'white')
%plot path
plot(x,y,'-','Color', [col_g col_g col_g])
hold on
% drawing squares
for ii= 1:length(i_sampling)
    x_s = x_samples(ii);
    y_s = y_samples(ii);
    t2_s = t_samples(ii);
    Px = x_s + cos(t2_s).*Px_a -sin(t2_s).*Py_a;
    Py = y_s + sin(t2_s).*Px_a+cos(t2_s).*Py_a;
    %center of mass
    plot(x_s,y_s,'o','Color', [col_g col_g col_g])
    %three grey body lines
    plot(Px([2,3,4,1]),Py([2,3,4,1]),'Color', [col_g col_g col_g])
    
    %heading line
    plot(Px([1,2]), Py([1,2]),'Color', [col_gh col_gh col_gh], 'LineWidth', head_LineWidth)
end
grid on
axis square
%% limit axes
    
xlim([-2*a, +2*a]+[min(x_samples),max(x_samples)]);
xticks( round(linspace(min(x_samples), max(x_samples), tick_spacing_n), 0))
ylim([-2*a, +2*a]+[min(y_samples),max(y_samples)]);
yticks( round(linspace(min(y_samples), max(y_samples), tick_spacing_n), 0))

% if x_samples(1) < 0
%     xlim([-2*a, +2*a]+[x_samples(1), 0]);
%     xticks( linspace(x_samples(1), 0, tick_spacing_n) )
% else
%     xlim([-2*a, +2*a]+[0, x_samples(1)]);
%     xticks( linspace(0, x_samples(1), tick_spacing_n) )
% end
% if y_samples(1) < 0
%     ylim([-2*a, +2*a]+[y_samples(1), 0]);
%     yticks( linspace(y_samples(1), 0, tick_spacing_n) )
% else
%     ylim([-2*a, +2*a]+[0, y_samples(1)]);
%     yticks( linspace(0, y_samples(1), tick_spacing_n) )
% end

%% 
% xlabel('X_{rel} [m]')
% ylabel('Y_{rel} [m]')


q = gca;
q.GridLineStyle = '--';
q.GridAlpha = 0.3;

%draw arrows
x1 = x_samples(1);
x2 = x_samples(2);

y1 = y_samples(1);
y2 = y_samples(2);
%              [figx figy] = dsxy2figxy([x1 y1],[x2 y2]) ;
%              arrowObj = annotation('arrow', [0.1 0.1], [0.5 0.5]);
%              set(arrowObj, 'Units', 'centimeters');
%              set(arrowObj, 'Position', [x1 y1 x2 y2]);
%              annotation(gcf,'arrow', figx,figy)

drawArrow = @(x,y, varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1), varargin{:});
hq = drawArrow([x1,x2],...
    [y1,y2], 'Color', [col_g col_g col_g], 'MaxHeadSize', 15);
%              U = hq.UData;
%              V = hq.VData;
%              X = hq.XData;
%              Y = hq.YData;
%              LineLength = 0.08;
%              headLength = 8;

%              for ii = 1:length(X)
%                  for ij = 1:length(X)
%
%                      headWidth = 5;
%                      ah = annotation('arrow',...
%                          'headStyle','cback1','HeadLength',headLength,'HeadWidth',headWidth);
%                      set(ah,'parent',gca);
%                      set(ah,'position',[X(ii,ij) Y(ii,ij) LineLength*U(ii,ij) LineLength*V(ii,ij)]);
%
%                  end
%              end

% or use annotation arrows
%
%              annotation(gcf,'arrow', xf,yf)
%              headWidth = 5;
%              ah = annotation('arrow',...
%                  'headStyle','cback1','HeadLength',headLength,'HeadWidth',headWidth);
%              set(ah,'parent',gca);
%              set(ah,'position',[x_arrow y_arrow LineLength*U(ii,ij) LineLength*V(ii,ij)]);



%              for ii= 1:n_sampling
%                  x_s = x_samples(ii);
%                  y_s = y_samples(ii);
%                  t2_s = theta2q2a_s(ii);
%              polyin = polyshape(Px,Py);
%              poly2(ii) = rotate(polyin,t2_s,[x_s y_s]);
%              end
%              plot(poly2)
%              axis equal

end