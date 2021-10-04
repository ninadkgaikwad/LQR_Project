function [omega_array] = omega_array_Creator(LowPower,HighPower)


%% Creating Logarithmically spaced values 

Vector_Power=LowPower:HighPower; % Vector of powers of 10

Required_Multiples=[1 2 3 4 5 6 7 8 9 10];

for ii=1:length(Vector_Power)-1 % For each Power of 10
    
    if (ii==1)
    
     Current_OmegaVector=10^(Vector_Power(ii))*Required_Multiples;
     
     omega_array=Current_OmegaVector;
     
    else
    
        Current_OmegaVector=10^(Vector_Power(ii))*Required_Multiples(2:end);

        omega_array=horzcat(omega_array,Current_OmegaVector);
    
    end
    
    
end

end

