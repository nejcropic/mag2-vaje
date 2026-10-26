classdef queueingSystemAnimator < simevents.SimulationObserver
    % queueingSystemAnimator Observer object for SimEvents animation
    %   
    % G. Music, 2020; based on sedemo_RestaurantAnimator.m
    
    properties
        mModel
        
        mFig;                   % figure handle
        mAx;                    % axis handle
        mTxtEntry;              % text handle displaying number of entities entered
        mTxtWaiting;            % text handle displaying number of entities waiting
        mTxtServiced;           % text handle displaying number of entities serviced
        mTxtExit;               % text handle displaying number of entities exited
        
        mTxtEntryTotal;         % text handle displaying total number of entities entered
        mTxtExitTotal;          % text handle displaying total number of entities exited
        
        dWaitPos    = 0;        % displacement position of last waiting
        dServicePos  = 0;       % displacement position of last serviced 
        dExitPos     = 0;       % displacement position of last exited

        queueColor = [0,0,0];
        serviceColor = [0,0,0];
    end
    
    properties (Constant, Access=private)
        
        cWaitPos = [25 50];     % X,Y position of entry
        cServicePos = [58,50];  % X,Y position of servicing area
        cExitPos = [85,50];     % X,Y position of exit
        
        sWaitPosX = 46;     % size of wait area
        sWaitPosY = 10;
        sServicePosX = 20;  % size of service area
        sServicePosY = 20;
        sExitPosX = 20;     % size of exit area
        sExitPosY = 20;

        posINCREMENT = 3;
        posDECREMENT = -3;
        
        INCREMENT = 1;
        DECREMENT = -1;
    end    
    
    methods
        
        function this = queueingSystemAnimator(modelname)
            % Constructor
            this@simevents.SimulationObserver(modelname);
            this.mModel = modelname;
        end
        
        function simStarted(this)
            % Called when simulation is started
            %
            % Initialize the animation canvas
            figure(1);
            h=gcf;
            o=findobj(gcf);
            delete(o(2:end));
            this.mFig = h;
            this.mAx = gca;
            set(this.mFig, 'toolbar', 'none');
            set(this.mFig, 'menubar', 'none');
            set(this.mAx, 'XTickLabel', '');
            set(this.mAx, 'YTickLabel', '');
            set(this.mAx, 'Box', 'on');
            set(this.mAx, 'TickLength', [0 0]);
            set(this.mAx, 'position', [0 0 1 1]);
            set(this.mAx, 'XLim', [0 100]);
            set(this.mAx, 'YLim', [0 100]);
            hold on;
            
            fsize = 12;
            
            % Static text annotations
            text(2,90, 'On entry', 'FontSize', 12);
            text(2,80, 'Total \newlineentered', 'FontSize', 12);
            text(27,90, 'Waiting', 'FontSize', 12);
            text(50,90, 'Serviced', 'FontSize', 12);
            text(77,90, 'On exit', 'FontSize', 12);
            text(77,80, 'Total \newlineexited', 'FontSize', 12);
            
            
            % Set up the numeric statistics text labels on the figure
            this.mTxtEntry = text(15,90, '0');
            this.mTxtWaiting = text(40,90, '0');
            this.mTxtServiced = text(65,90, '0');
            this.mTxtExit = text(88,90, '0');
            
            set(this.mTxtEntry, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);
            set(this.mTxtWaiting, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);
            set(this.mTxtServiced, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);
            set(this.mTxtExit, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);

            this.mTxtEntryTotal = text(15,80, '0');
            this.mTxtExitTotal = text(88,80, '0');
            
            set(this.mTxtEntryTotal, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);
            set(this.mTxtExitTotal, 'Color', [0.8500 0.3250 0.0980], 'FontWeight', 'bold', 'FontSize', fsize);
            
            % Reset displacement 
            this.dWaitPos = 20;      % displacement position of last waiting
            this.dServicePos = 0;   % displacement position of last service
            this.dExitPos = 0;      % displacement position of last exit
            
            % draw queue area
            patchSizeX = this.sWaitPosX;
            patchSizeY = this.sWaitPosY;
            cPatch=this.cWaitPos;
            pX=[cPatch(1)-patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)-patchSizeX/2];
            pY=[cPatch(2)+patchSizeY/2,cPatch(2)+patchSizeY/2,cPatch(2)-patchSizeY/2,cPatch(2)-patchSizeY/2];
            patch(pX,pY,'y');
            
            % draw machine
            patchSizeX = this.sServicePosX;
            patchSizeY = this.sServicePosY;
            cPatch=this.cServicePos;
            pX=[cPatch(1)-patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)-patchSizeX/2];
            pY=[cPatch(2)+patchSizeY/2,cPatch(2)+patchSizeY/2,cPatch(2)-patchSizeY/2,cPatch(2)-patchSizeY/2];
            patch(pX,pY,'c');

            % draw storage area
            patchSizeX = this.sExitPosX;
            patchSizeY = this.sExitPosY;
            cPatch=this.cExitPos;
            pX=[cPatch(1)-patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)+patchSizeX/2,cPatch(1)-patchSizeX/2];
            pY=[cPatch(2)+patchSizeY/2,cPatch(2)+patchSizeY/2,cPatch(2)-patchSizeY/2,cPatch(2)-patchSizeY/2];
            patch(pX,pY,'y');
            
        end
        
        % -----------------------------------------------------------------
        function simTerminating(this)
            % Called when simulation is terminating
            %
            % After the simulation terminates, ...
            
            % Reset displacement 
            this.dWaitPos = 10;      % displacement position of last waiting
            this.dServicePos = 0;   % displacement position of last service
            this.dExitPos = 0;      % displacement position of last exit

        end
        
        
        % -----------------------------------------------------------------
        function blks = getBlocksToNotify(this)
            % Return list of blocks to observe in the model
            %
            % For this example, we are only interested in the following
            % blocks as they are sufficient for us to know all events of
            % interest
            blks = { ...
                [this.mModel '/Vrsta'], ...
                [this.mModel '/Vrtalnik'] ...
                };
        end

        % -----------------------------------------------------------------
        function postEntry(this, evSrc, evData) %#ok<INUSL>
            % Override to specify listener for entry into a storage (queue/server)

            %entity = evData.CurrentEntity;
            
            % block list (block order is defined by getBlocksToNotify function)
            % Queue must be first, Server second
            blks = getBlocksToNotify(this);

            if strcmp(evData.Block.BlockPath, blks{1}) % Queue

                % Increment the entry statistic
                this.updateStats(this.mTxtEntryTotal, this.INCREMENT);
                
                % Create a new "dot" on the figure at the waiting position
                h = plot(this.cWaitPos(1)+this.dWaitPos, this.cWaitPos(2), '.');
                set(h, 'MarkerSize', 32, 'Tag', 'waiting');
                
                % Change displacement for the next entry
                this.dWaitPos = this.dWaitPos + this.posDECREMENT;
                
                % Increment the waiting statistic
                this.updateStats(this.mTxtWaiting, this.INCREMENT);
                
            elseif strcmp(evData.Block.BlockPath, blks{2}) % Server

                % Delete a "dot" on the figure at the wait position
                h=findobj('Tag','waiting');
                if length(h)==1
                    XData=get(h,'XData');
                else
                    XData=cell2mat(get(h,'XData'));
                end
                [~,first]=max(XData);
                this.queueColor=get(h(first),'Color');
                delete(h(first));
                
                % Move remaining "dots"
                for i=1:length(h)
                    if i~= first
                        set(h(i),'XData',XData(i)+this.posINCREMENT);
                    end
                end
                
                % Change displacement for the next entry
                this.dWaitPos = this.dWaitPos + this.posINCREMENT;
                
                % Create a new "dot" on the figure at the servicing position
                h = plot(this.cServicePos(1)+this.dServicePos, this.cServicePos(2), '.');
                set(h, 'MarkerSize', 32, 'Color', this.queueColor, 'Tag', 'serviced');

                % Increment the service statistic
                this.updateStats(this.mTxtServiced, this.INCREMENT);
                
            end
            
        end
        
        % -----------------------------------------------------------------
        function preExit(this, ~, evData)
            % Override to specify listener for exit from a storage (queue/server)
            % evData contains block, storage, and entity handles

            %entity = evData.CurrentEntity;
            
            % seznam možnih blokov (vrstni red je doloèen s funkcijo getBlocksToNotify)
            blks = getBlocksToNotify(this);
            
            if strcmp(evData.Block.BlockPath, blks{1}) % Queue
                
                % Decrement waiting statistic
                this.updateStats(this.mTxtWaiting, this.DECREMENT);
                
            elseif strcmp(evData.Block.BlockPath, blks{2}) % Server
                
                % Delete a "dot" on the figure at the service position
                h=findobj('Tag','serviced');
                this.serviceColor=get(h,'Color');
                delete(h);
                
                % Create a new "dot" on the figure at the exit position
                exitPos = this.getRandExitPos();
                h = plot(exitPos(1), exitPos(2), '.');
                set(h, 'MarkerSize', 32, 'Color', this.serviceColor);
                
                % Decrement service statistic
                this.updateStats(this.mTxtServiced, this.DECREMENT);
                
                % Increment the exiting statistic
                this.updateStats(this.mTxtExitTotal, this.INCREMENT);
                
            end
            
        end
        
    end

    % =====================================================================
    methods (Access = private)

        % -----------------------------------------------------------------
        function pos = getRandExitPos(this)
            % Helper function to generate a random point in the storage
            % area so we can show entities stored at random positions
            
            exitCenter = this.cExitPos;
            exitSizeX = this.sExitPosX;
            exitSizeY = this.sExitPosY;
            xrand = randi(exitSizeX-1)-exitSizeX/2;
            yrand = randi(exitSizeY-1)-exitSizeY/2;
            pos = exitCenter + [xrand, yrand];
        end
        
        % -----------------------------------------------------------------
        function updateStats(~, htext, val)
            % Increment or decrement statistic text on the figure
            
            curCount = str2double(get(htext, 'String'));
            set(htext, 'String', num2str(curCount+val));
        end
        
    end    
        
end

