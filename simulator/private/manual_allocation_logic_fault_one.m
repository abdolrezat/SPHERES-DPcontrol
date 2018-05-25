function [f0,f1,f6,f7] = manual_allocation_logic_fault_one(Fx,My,x,w, max_T, faultid)
persistent priorityw
if(isempty(priorityw))
    priorityw = 0;
end

Fx_threshhold = 0.12;
My_threshhold_1 = 0.12;
%
% if( abs(w) > deg2rad(10) )
%     priorityw = 1;
% end
%
% if( abs(w) < deg2rad(0.3) )
%     priorityw = 0;
% end
%
% if( abs(x) < 0.5 )
%     priorityw = 1;
% end

if(faultid == 0)
    
if( Fx > Fx_threshhold)
    if(abs(My) < My_threshhold_1)
        %             Fx = 0.26;
        %             My = 0;
        f0 = 0; %%%
        f1 = max_T;
        f6 = 0;
        f7 = 0;
    else
        if(My > 0)
            %             Fx = 0.13;
            %             My = 0.13;
            f0 = 0;  %%%
            f1 = 0.13;
            f6 = 0;
            f7 = 0;
        elseif(My < 0)
            
            %             Fx = 0.13;
            %             My = -0.13;
            f0 = 0;
            f1 = max_T;
            f6 = 0;
            f7 = 0;
        end
    end
    
elseif( Fx < -Fx_threshhold)
    if(My >= 0)
        if(My < My_threshhold_1)
            %             Fx = -0.26;
            %             My = 0;
            f0 = 0;
            f1 = 0;
            f6 = max_T;
            f7 = max_T;
        else
            %             Fx = -0.13;
            %             My = 0.13;
            f0 = 0;
            f1 = 0;
            f6 = 0;
            f7 = max_T;
        end
    else
        if(abs(My) < My_threshhold_1)
            %             Fx = -0.26;
            %             My = 0;
            f0 = 0;
            f1 = 0;
            f6 = max_T;
            f7 = max_T;
        else
            %             Fx = -0.13;
            %             My = -0.13;
            f0 = 0;
            f1 = 0;
            f6 = max_T;
            f7 = 0;
        end
    end
    
elseif( abs(Fx) < 0.5 )
    %     Fx = 0;
    if( Fx >= 0)
        if( abs(My) < My_threshhold_1)
            %         My = 0;
            f0 = 0; 
            f1 = 0;
            f6 = 0;
            f7 = 0;
        else
            %         My = (abs(My)/My)*0.13;
            if(My < My_threshhold_1*2)
                if(My > 0)
                    f0 = 0; %%%
                    f1 = 0;
                    f6 = 0;
                    f7 = max_T;
                else
                    f0 = 0;
                    f1 = max_T;
                    f6 = 0;
                    f7 = 0;
                end
            elseif(My > My_threshhold_1*2)
                if(My > 0)
                    f0 = 0; %%%
                    f1 = 0;
                    f6 = 0;
                    f7 = max_T;
                else
                    f0 = 0;
                    f1 = max_T;
                    f6 = max_T;
                    f7 = 0;
                end
            end
        end
        
    else
        if( abs(My) < My_threshhold_1)
            %         My = 0;
            f0 = 0;
            f1 = 0;
            f6 = 0;
            f7 = 0;
        else
            %         My = (abs(My)/My)*0.13;
            if(My < My_threshhold_1*2)
                if(My > 0)
                    f0 = 0;
                    f1 = 0;
                    f6 = 0;
                    f7 = max_T;
                else
                    f0 = 0;
                    f1 = 0;
                    f6 = max_T;
                    f7 = 0;
                end
            elseif(My > My_threshhold_1*2)
                if(My > 0)
                    f0 = 0;
                    f1 = 0;
                    f6 = 0;
                    f7 = max_T;
                else
                    f0 = 0;
                    f1 = max_T;
                    f6 = max_T;
                    f7 = 0;
                end
            end
        end
        
    end
end
    
    
end



