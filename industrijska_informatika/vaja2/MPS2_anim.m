function MPS2_anim(f,u)

% assumed input components
% u(1) position of cylinder
% u(2) position of cylinder
% u(3) position of cylinder
% u(4) insert workpiece 
% u(5) measurement

% animation
persistent v_ref

if nargin<2
    mode=1;
else
    mode=2;
    if length(u)<1
        eposition = 0;
    else
        eposition = u(1);
    end
    if length(u)<2
        c1position = 0;
    else
        c1position = u(2);
    end
    if length(u)<3
        c2position = 0;
    else
        c2position = u(3);
    end
    if length(u)<4
        storage = 0;
    else
        storage = u(4);
    end
    if length(u)<5
        measurement = 0;
    else
        measurement = u(5);
    end
end
switch mode
    case 1
        % initialisation
        figure(f);
        clf;
        shown_range=400;
        axis([-10 shown_range -10 shown_range])
        % figure properties
        set(gca,'XTick',0:50:shown_range, ...
            'ButtonDownFcn','set_param(bdroot,''SimulationCommand'',''stop'')');
        %
        % draw elevator
        ref_position=[200,0];
        % base part 1
        base=[-100 100 100 -100 -100;
                0 0 20 20 0];
        line(ref_position(1)+base(1,:),ref_position(2)+base(2,:));
        % base part 2
        base=[-10 10 10 -10 -10;
                20 20 350 350 20];
        line(ref_position(1)+base(1,:),ref_position(2)+base(2,:));
        % moving part
        ref_position=[200,20];
        arm=[-50 25 25 -50;
            0 0 10 10];
        platform=patch(ref_position(1)+arm(1,:),ref_position(2)+arm(2,:),'b','Tag','Elevator');
        %
        % draw piston 1
        ref_position=[200,38];
        % base part 
        base=[25 -10 -10 25 25;
            -8 -8 8 8 -8];
        line(ref_position(1)+base(1,:),ref_position(2)+base(2,:),'Tag','Piston1b');
        % moving part
        ref_position=[190,38];
        hold_position=[210,0];
        piston=[0 -10 -10 0;
            -5 -5 5 5];
        patch(ref_position(1)+piston(1,:),ref_position(2)+piston(2,:),'b','Tag','Piston1');
        %
        % draw piston 2
        ref_position=[167.5,350];
        % holder
        holder=[8 22.5 22.5 8 8;
            0 0 -30 -30 0];
        line(ref_position(1)+holder(1,:),ref_position(2)+holder(2,:));
        % base part 
        base=[-8 8 8 -8 -8;
            0 0 -50 -50 0];
        line(ref_position(1)+base(1,:),ref_position(2)+base(2,:));
        % moving part
        ref_position=[167.5,300];
        hold_position=[210,0];
        piston=[-5 -5 5 5;
                0 -10 -10 0];
        patch(ref_position(1)+piston(1,:),ref_position(2)+piston(2,:),'b','Tag','Piston2');
        %
        % store initial position vertices into a persistent variable
        v_ref=get(platform,'Vertices');
        % display initial measurement output
        text(100,300,int2str(0),'Tag','Height');
    case 2
        % adjustment of object positions with respect to u value
        figure(f);
        %
        % elevator
        platform=findobj('Tag','Elevator');
        % get initial position vertices from persistent variable
        v_platform=v_ref;
        base_point=[200,20];
        % adjust points
        v_platform(:,2)=v_ref(:,2)+eposition;
        % show adjusted arm vertices
        set(platform,'Vertices',v_platform);
        % move piston1 with the arm
        % base part
        piston1b=findobj('Tag','Piston1b');
        % % get YData from figure
        % ydata=get(piston1b,'YData');
        ref_position=[200,38];
        ref_vert_offset=[-8 -8 8 8 -8];
        % adjust endpoints
        ydata=ref_position(2)+ref_vert_offset+eposition;
        % show adjusted YData
        set(piston1b,'YData',ydata);
        % moving part
        piston1=findobj('Tag','Piston1');
        % get piston vertices from figure
        v=get(piston1,'Vertices');
        ref_position=[190,38];
        ref_vert_offset=[-5 -5 5 5]';
        % adjust endpoints
        v(:,2)=ref_position(2)+ref_vert_offset+eposition;
        % show adjusted piston vertices
        set(piston1,'Vertices',v);
        %
        % piston 1
        piston1=findobj('Tag','Piston1');
        % get piston vertices from figure
        v=get(piston1,'Vertices');
        % endposition of retracted piston
        pos0=180;
        % endposition of the extended piston
        hold_position=[140,0];
        % find endpoints
        vi=find(v(:,1)==min(v(:,1)));
        % adjust endpoints
        move=c1position;
        v(vi,1)=pos0-move;
        % show adjusted piston vertices
        set(piston1,'Vertices',v);
        %
        % piston 2
        piston2=findobj('Tag','Piston2');
        % get piston vertices from figure
        v=get(piston2,'Vertices');
        % endposition of retracted piston
        pos0=290;
        % endposition of the extended piston
        hold_position=[167.5,250];
        % find endpoints
        vi=find(v(:,2)==min(v(:,2)));
        % adjust endpoints
        move=c2position;
        v(vi,2)=pos0-move;
        % show adjusted piston vertices
        set(piston2,'Vertices',v);
        %
        % storage
        %
        % workpiece at the elevator
        ref_wkpos=[180-25,30];
        wkpos(1)=ref_wkpos(1)-c1position;
        wkpos(2)=ref_wkpos(2)+eposition;
        wkp=[0 25 25 0;
            0 0 20 20];
        wk1=findobj('Tag','Workpiece1');
        if isempty(wk1)
            if storage && c1position==0
                wk1=patch(wkpos(1)+wkp(1,:),wkpos(2)+wkp(2,:),'r','Tag','Workpiece1');
                % height
                type=randi(3);
                %noise=randi(200)-100;
                noise=0;
                switch type
                    case 1
                        base_ht=7337;
                    case 2
                        base_ht=9965;
                    case 3
                        base_ht=12389;
                end
                wk1.UserData=base_ht+noise;
            end
        else
            if ~storage
                delete(wk1);
            else
                v=wk1.Vertices;
                if c1position>0
                    v(:,1)=wkpos(1)+wkp(1,:);
                end
                if eposition>0
                    v(:,2)=wkpos(2)+wkp(2,:);
                end
                % show adjusted workpiece vertices
                set(wk1,'Vertices',v);
            end
        end
        %
        % measurement - samo prikažem zunanji signal!
        %
        height=findobj('Tag','Height');
        height.String=int2str(measurement);
 otherwise
        disp('Wrong mode');
end