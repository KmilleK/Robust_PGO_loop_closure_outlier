function testing_cvx(measurements,n,w_r)

 cvx_begin sdp
    
        variable X(2*n,2*n) symmetric;
        objective=0;
        for i=1:length(measurements)
            error=X(2*measurements(i).i-(2-1):2*measurements(i).i,2*measurements(i).j-(2-1):2*measurements(i).j)-measurements(i).R;
            %objective=objective+huber(sum(sum(error)));   
            objective=objective+huber_fro(error);
        end
        minimize (objective)
        X >= 0;
        for k=1:n 
           X(2*k-1:2*k,2*k-1:2*k)==eye(2);
        end
cvx_end

end