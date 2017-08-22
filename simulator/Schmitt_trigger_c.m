classdef Schmitt_trigger_c < handle
    %SCHMITT_TRIGGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        u_persistent
        Uout
        Uon
        Uoff
    end
    
    methods
        function this = Schmitt_trigger_c(Uout,Uon,Uoff)
                this.u_persistent = 0;
                this.Uout = Uout;
                this.Uon = Uon;
                this.Uoff = Uoff;
        end
        
        function out = signal_update(obj, Uin)

            if( abs(Uin) < obj.Uoff )
                obj.u_persistent = 0;
            end
            
            if( abs(Uin) > obj.Uon )
                if(Uin > obj.Uon)
                    obj.u_persistent = obj.Uout;
                else
                    obj.u_persistent = -obj.Uout;
                end
            end
            out = obj.u_persistent;
        end
        
    end
    
end
    
