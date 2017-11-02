%%
magnitude=5;
fixed_frame=[magnitude magnitude magnitude 1 ; 0 0 0 1];

    plot3(fixed_frame(:,1),[0;0],[0;0],'r','LineWidth',3)
    hold on
    plot3([0;0],fixed_frame(:,2),[0;0],'g','LineWidth',3)
    plot3([0;0],[0;0],fixed_frame(:,3),'k','LineWidth',3)
    axis equal
    scatter3(leg.h1.p(1),leg.h1.p(2),leg.h1.p(3),'filled','y','MarkerEdgeColor','k');
    scatter3(leg.h2.p(1),leg.h2.p(2),leg.h2.p(3),'filled','y','MarkerEdgeColor','k');
    fill3([leg.h1.p(1);leg.h2.p(1)],[leg.h1.p(2);leg.h2.p(2)],[leg.h1.p(3);leg.h2.p(3)],'b');
    
   scatter3(leg.foot(1),leg.foot(2),leg.foot(3),'filled','k','MarkerEdgeColor','k');
   
   scatter3(leg.h1.k.p(1),leg.h1.k.p(2),leg.h1.k.p(3), 'filled','g','MarkerEdgeColor','k');
   scatter3(leg.h2.k.p(1),leg.h2.k.p(2),leg.h2.k.p(3), 'filled','g','MarkerEdgeColor','k');
   
   plot3([leg.h1.p(1),leg.h1.k.p(1),leg.foot(1)],[leg.h1.p(2),leg.h1.k.p(2),leg.foot(2)],[leg.h1.p(3),leg.h1.k.p(3),leg.foot(3)],'LineWidth',3,'color','b')
   plot3([leg.h2.p(1),leg.h2.k.p(1),leg.foot(1)],[leg.h2.p(2),leg.h2.k.p(2),leg.foot(2)],[leg.h2.p(3),leg.h2.k.p(3),leg.foot(3)],'LineWidth',3,'color','b')
    
   axis([-12 12 -12 12 -20 12]) 
%     drawnow
    hold off
    