%% reference trajectory generator
function ref_at_t = F_ref_at_t(t,ref_type)
if strcmp(ref_type,'step_zero21_at5')    
    ref_at_t = 0*(t<5) + 1*(t>=5);
else strcmp(ref_type,'step_zero21_at5_back20_at15')    
    ref_at_t = 1*(t>=5).*(t<15);    
end;
    
    
