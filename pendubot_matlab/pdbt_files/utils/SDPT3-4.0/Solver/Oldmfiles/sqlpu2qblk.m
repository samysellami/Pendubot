%%*************************************************************************
%% sqlp: main solver 
%%
%%*************************************************************************
%% SDPT3: version 3.1
%% Copyright (c) 1997 by
%% K.C. Toh, M.J. Todd, R.H. Tutuncu
%% Last Modified: 16 Sep 2004
%%*************************************************************************

  function [obj,X,y,Z,info,runhist] = sqlpmain(blk,At,C,b,par,parbarrier,X0,y0,Z0);

   global spdensity  printlevel  msg
   global solve_ok  use_LU  exist_analytic_term  
   global schurfun  schurfun_par 
%%
   randstate = rand('state');  randnstate = randn('state');
   rand('state',0);   randn('state',0);
%%
   matlabversion = par.matlabversion;
   vers          = par.vers;
   predcorr      = par.predcorr;
   gam           = par.gam; 
   expon         = par.expon;
   gaptol        = par.gaptol;
   inftol        = par.inftol;
   steptol       = par.steptol;
   maxit         = par.maxit;
   printlevel    = par.printlevel;
   stoplevel     = par.stoplevel;
   scale_data    = par.scale_data;
   spdensity     = par.spdensity;
   rmdepconstr   = par.rmdepconstr;
   cachesize     = par.cachesize; 
   smallblkdim   = par.smallblkdim;
   schurfun      = par.schurfun;
   schurfun_par  = par.schurfun_par;
   ublksize      = par.ublksize; 
%%
   tstart = cputime; 
   X = X0; y = y0; Z = Z0; 
   for p = 1:size(blk,1)
      if strcmp(blk{p,1},'u'); Z{p} = zeros(blk{p,2},1); end
   end
%%
%%-----------------------------------------
%% convert unrestricted blk to socp blk. 
%%-----------------------------------------
%%
   convertlen = 0; 
   [blk,At,C,X,Z,u2lblk,ublkidx] = sqlpu2lblk(blk,At,C,X,Z,par,convertlen);
   if any(u2lblk)
      b = [b; 0; 0.5; 0.5; 0]; 
      y = [y; zeros(4,1)];
      for p = 1:size(blk,1) 
         pblk = blk(p,:); 
         if (u2lblk(p) == 1) 
            n = blk{p,2}+1;   
            blk{p,1} = 'q';  blk{p,2} = [n,3]; 
            parbarrier{p} = [0,0]; 
            n2 = size(At{p},1); m = size(At{p},2); 
            At{p} = [sparse(1,m),-1,0,0,0; At{p},sparse(n2,4); ...
                     sparse(3,m+1),speye(3,3)]; 
            C{p} = [0; C{p}; zeros(3,1)];
            msg  = '*** convert ublk to qblk'; 
            if (printlevel); fprintf(' %s',msg); end
            b2 = 1 + abs(b)';  
            normCtmp = 1+norm(C{p});
            normAtmp = 1+sqrt(sum(At{p}.*At{p}));
            if (par.startpoint == 1)
               constX = max([1,b2./normAtmp])*sqrt(n); 
               constZ = max([sqrt(n),normAtmp,normCtmp]);
               X{p} = constX*[1; 1e-10*rand(n-1,1); 1;0;0];
               Z{p} = constZ*[1; 1e-10*rand(n-1,1); 1;0;0];
            end
            tau  = max(1,norm(C{p})); 
         else
            At{p} = [At{p}, sparse(size(At{p},1),4)]; 
         end
      end
      numblk = size(blk,1); 
      blk{numblk+1,1} = 'l'; blk{numblk+1,2} = 2; 
      At{numblk+1,1}  = [sparse(1,m+1),-0.5,0.5,0; sparse(1,m),1,0,0,-1]; 
      C{numblk+1,1}   = tau*[1; 1e-6]; 
      X{numblk+1,1}   = 1e2*tau*[1;1]; 
      Z{numblk+1,1}   = 1e2*tau*[1;1];       
      parbarrier{numblk+1} = 0; 
      u2lblk(numblk+1) = 0; 
   end
%%-----------------------------------------
%% check whether {A1,...,Am} is 
%% linearly independent. 
%%-----------------------------------------
%%
   m0 = length(b); 
   [At,b,y,indeprows,par.depconstr,feasible,par.AAt] = ...
    checkdepconstr(blk,At,b,y,rmdepconstr);
   if (~feasible)
      obj = []; X = cell(size(blk,1),1); y = []; Z = cell(size(blk,1),1); 
      runhist = [];      
      msg = 'SQLP is not feasible'; 
      if (printlevel); fprintf('\n %s \n',msg); end
      return;
   end
   par.normAAt = norm(par.AAt,'fro'); 
%%
%%-----------------------------------------
%% scale SQLP data. Note: must be done only 
%% after checkdepconstr
%%-----------------------------------------
%%
   normA2 = 1+ops(At,'norm'); 
   normb2 = 1+norm(b); 
   normC2 = 1+ops(C,'norm'); 
   normX0 = 1+ops(X0,'norm'); 
   normZ0 = 1+ops(Z0,'norm'); 
   if (scale_data)
      [At,C,b,normA,normC,normb,X,y,Z] = scaling(blk,At,C,b,X,y,Z);
   else
      normA = 1; normC = 1; normb = 1; 
   end 
%%
%%-----------------------------------------
%% find the combined list of non-zero 
%% elements of Aj, j = 1:k, for each k. 
%% IMPORTANT NOTE: Ak, C are permuted.
%%-----------------------------------------
%% 
   par.numcolAt = length(b); 
   [At,C,X,Z,par.permA,par.permZ] = sortA(blk,At,C,b,X,Z);
   [par.isspA,par.nzlistA,par.nzlistAsum,par.isspAy,par.nzlistAy] = nzlist(blk,At,par);
%%
%%-----------------------------------------
%% create an artifical non-negative block 
%% for a purely log-barrier problem
%%-----------------------------------------
%%
   numblkold = size(blk,1);  
   nn = 0; 
   for p = 1:size(blk,1);
      pblk = blk(p,:);  
      idx = find(parbarrier{p}==0); 
      if ~isempty(idx); 
         nn = nn + length(idx); 
      end
   end
   if (nn==0)
      analytic_prob = 1; 
      numblk = size(blk,1)+1; 
      blk{numblk,1} = 'l'; blk{numblk,2} = 1; 
      At{numblk,1} = sparse(1,length(b)); 
      C{numblk,1} = 1; 
      X{numblk,1} = 1e3; 
      Z{numblk,1} = 1e3;
      parbarrier{numblk,1} = 0; 
      u2lblk(numblk,1) = 0;
      nn = nn + 1; 
   else
      analytic_prob = 0;       
   end
%%
   exist_analytic_term = 0; 
   for p = 1:size(blk,1);
      idx = find(parbarrier{p} > 0); 
      if ~isempty(idx); 
         exist_analytic_term = 1; 
      end
   end
%%-----------------------------------------
%% initialization
%%-----------------------------------------
%%
   EE = ops(blk,'identity');
   normE2 = ops(EE,'norm'); Zpertold = 1; 
   for p = 1:size(blk,1) 
      normCC(p) = 1+ops(C(p),'norm');
      normEE(p) = 1+ops(EE(p),'norm'); 
   end
   [Xchol,indef(1)] = blkcholfun(blk,X); 
   [Zchol,indef(2)] = blkcholfun(blk,Z); 
   if any(indef)
      msg = 'sqlp stop: X or Z not positive definite'; 
      if (printlevel); fprintf('\n  %s\n',msg); end
      info.termcode = -3;
      info.msg1 = msg;
      obj = []; X = cell(size(blk,1),1); y = []; Z = cell(size(blk,1),1); 
      runhist = [];      
      return;
   end 
   AX = AXfun(blk,At,par.permA,X); 
   rp = b-AX;
   ZpATy = ops(Z,'+',Atyfun(blk,At,par.permA,par.isspAy,y));
   ZpATynorm = ops(ZpATy,'norm');
   Rd = ops(C,'-',ZpATy);
   objadd0 = 0; 
   if (scale_data)
      for p = 1:size(blk,1)
         pblk = blk(p,:); 
         objadd0 = objadd0 + sum(parbarrier{p}.*pblk{2})*log(normA{p}); 
      end
   end
   objadd = blkbarrier(blk,X,Z,Xchol,Zchol,parbarrier) + objadd0;
   obj = (normb*normC)*[blktrace(blk,C,X), b'*y] + objadd;      
   gap = (normb*normC)*blktrace(blk,X,Z) - diff(objadd); 
   relgap = gap/(1+sum(abs(obj)));
   prim_infeas = norm(rp)/normb2;
   dual_infeas = ops(Rd,'norm')/normC2;
   infeas = max(prim_infeas,dual_infeas); 
   if (scale_data)
      infeas_org(1) = prim_infeas*normb;
      infeas_org(2) = dual_infeas*normC;
   else
      infeas_org = [0,0]; 
   end
   trXZ = blktrace(blk,X,Z,parbarrier); 
   if (nn > 0); mu  = trXZ/nn; else; mu = gap/ops(X,'getM'); end
   normX = ops(X,'norm'); 
%%   
   termcode = 0; restart = 0; 
   pstep = 1; dstep = 1; pred_convg_rate = 1; corr_convg_rate = 1;
   prim_infeas_bad = 0;  prim_infeas_min = prim_infeas; 
   dual_infeas_bad = 0;  dual_infeas_min = dual_infeas; 
   homRd = inf; homrp = inf; dy = zeros(length(b),1);    
   msg = []; msg2 = []; msg3 = [];
   runhist.pobj = obj(1);
   runhist.dobj = obj(2); 
   runhist.gap  = gap;
   runhist.relgap  = relgap;
   runhist.pinfeas = prim_infeas;
   runhist.dinfeas = dual_infeas;
   runhist.infeas  = infeas;  
   runhist.step    = 0; 
   runhist.normX   = normX; 
   runhist.cputime = cputime-tstart; 
   ttime.preproc   = runhist.cputime; 
   ttime.pred = 0; ttime.pred_pstep = 0; ttime.pred_dstep = 0; 
   ttime.corr = 0; ttime.corr_pstep = 0; ttime.corr_dstep = 0; 
   ttime.pchol = 0; ttime.dchol = 0; ttime.misc = 0; 
%%
%%-----------------------------------------
%% display parameters and initial info
%%-----------------------------------------
%%
   if (printlevel >= 2)
      fprintf('\n********************************************');
      fprintf('***********************\n');
      fprintf('   SDPT3: Infeasible path-following algorithms'); 
      fprintf('\n********************************************');
      fprintf('***********************\n');
      [hh,mm,ss] = mytime(ttime.preproc); 
      if (printlevel>=3)       
         fprintf(' version  predcorr  gam  expon  scale_data\n');
         if (vers == 1); fprintf('   HKM '); elseif (vers == 2); fprintf('    NT '); end
         fprintf('     %1.0f      %4.3f',predcorr,gam);
         fprintf('   %1.0f        %1.0f    %1.0f\n',expon,scale_data); 
         fprintf('\nit pstep dstep pinfeas dinfeas  gap')
         fprintf('      mean(obj)   cputime\n');
         fprintf('------------------------------------------------');
         fprintf('-------------------\n');
         fprintf('%2.0f|%4.3f|%4.3f|%2.1e|%2.1e|',0,0,0,prim_infeas,dual_infeas);
         fprintf('%2.1e|%- 7.6e| %s:%s:%s|',gap,mean(obj),hh,mm,ss);
      end
   end
%%
%%---------------------------------------------------------------
%% start main loop
%%---------------------------------------------------------------
%%
   param.termcode    = termcode; 
   param.iter        = 0; 
   param.normA       = normA; 
   param.normb       = normb;
   param.normC       = normC;
   param.normX0      = normX0; 
   param.normZ0      = normZ0; 
   param.m0          = m0;
   param.indeprows   = indeprows;
   param.prim_infeas_bad = prim_infeas_bad; 
   param.prim_infeas_min = prim_infeas_min; 
   param.dual_infeas_bad = dual_infeas_bad; 
   param.dual_infeas_min = dual_infeas_min; 
   param.gaptol      = gaptol;
   param.inftol      = inftol; 
   param.maxit       = maxit;
   param.scale_data  = scale_data;
   param.printlevel  = printlevel; 
   param.ublksize    = ublksize; 
%%
   for iter = 1:maxit;
  
      tstart  = cputime;  
      timeold = tstart;
      update_iter = 0; breakyes = 0; pred_slow = 0; corr_slow = 0; step_short = 0; 
      par.parbarrier = parbarrier; 
      par.iter    = iter; 
      par.obj     = obj; 
      par.relgap  = relgap; 
      par.pinfeas = prim_infeas; 
      par.dinfeas = dual_infeas;
      par.y       = y; 
      par.dy      = dy; 
      par.normX   = normX; 
      par.ZpATynorm = ZpATynorm; 
      Xold = X; yold = y; Zold = Z;
      if (any(u2lblk))
         if (max(relgap,infeas) < 1e-3)
            tau = 0.5*tau; 
	 elseif (max(relgap,infeas) < 1)
            tau = 0.8*tau; 
	 else
	    tau = 0.9*tau; 
         end
         C{end}(1) = tau;   
         fprintf(' %3.2e',tau);  
         ZpATy = ops(Z,'+',Atyfun(blk,At,par.permA,par.isspAy,y));
         Rd = ops(C,'-',ZpATy); 
      end      
      if (iter == 1 | restart); Cpert = min(1,normC2/ops(EE,'norm')); end
      if (runhist.dinfeas(1) > 1e-3) & (~exist_analytic_term) ...
         & (relgap > 1e-4) 
         if (par.normX > 5e3 & iter < 20)
            Cpert = Cpert*0.5; 
         elseif (par.normX > 5e2 & iter < 20); 
            Cpert = Cpert*0.3; 
         else; 
            Cpert = Cpert*0.1; 
         end
         Rd = ops(Rd,'+',EE,Cpert); 
      end
%%---------------------------------------------------------------
%% predictor step.
%%---------------------------------------------------------------
%%
      if (predcorr)
         sigma = 0; 
      else 
         sigma = 1-0.9*min(pstep,dstep); 
         if (iter == 1); sigma = 0.5; end; 
      end
      sigmu = cell(size(blk,1),1);
      for p = 1:size(blk,1)
         sigmu{p} = max(sigma*mu, parbarrier{p}');  
      end
      invXchol = cell(size(blk,1),1); 
      invZchol = ops(Zchol,'inv'); 
      if (vers == 1);
         [par,dX,dy,dZ,coeff,L,hRd] = ...
          HKMpred(blk,At,par,rp,Rd,sigmu,X,Z,invZchol);
      elseif (vers == 2);
         [par,dX,dy,dZ,coeff,L,hRd] = ...
          NTpred(blk,At,par,rp,Rd,sigmu,X,Z,Zchol,invZchol);
      end
      if (solve_ok <= 0)
         msg = 'sqlp stop: difficulty in computing predictor directions'; 
         if (printlevel); fprintf('\n  %s',msg); end
         runhist.pinfeas(iter+1) = runhist.pinfeas(iter); 
         runhist.dinfeas(iter+1) = runhist.dinfeas(iter); 
         runhist.relgap(iter+1)  = runhist.relgap(iter); 
         runhist.cputime(iter+1) = cputime-tstart; 
         termcode = -4;
         break; %% do not ues breakyes = 1
      end
      timenew = cputime;
      ttime.pred = ttime.pred + timenew-timeold; timeold = timenew; 
%%
%%-----------------------------------------
%% step-lengths for predictor step
%%-----------------------------------------
%%
      if (gam == 0) 
         gamused = 0.9 + 0.09*min(pstep,dstep); 
      else
         gamused = gam;
      end 
      [Xstep,invXchol] = steplength(blk,X,dX,Xchol,invXchol); 
      pstep = min(1,gamused*full(Xstep));
      timenew = cputime; 
      ttime.pred_pstep = ttime.pred_pstep + timenew-timeold; timeold = timenew;
      Zstep = steplength(blk,Z,dZ,Zchol,invZchol); 
      dstep = min(1,gamused*full(Zstep));
      trXZnew = trXZ + pstep*blktrace(blk,dX,Z,parbarrier) ...
                 + dstep*blktrace(blk,X,dZ,parbarrier) ...
                 + pstep*dstep*blktrace(blk,dX,dZ,parbarrier);
      if (nn > 0); mupred  = trXZnew/nn; else; mupred = 1e-16; end
      mupredhist(iter) = mupred;
      timenew = cputime;        
      ttime.pred_dstep = ttime.pred_dstep + timenew-timeold; timeold = timenew;
%%
%%-----------------------------------------
%%  stopping criteria for predictor step.
%%-----------------------------------------
%%
      if (min(pstep,dstep) < steptol) & (stoplevel) & (iter > 10)
         msg = 'sqlp stop: steps in predictor too short';
         if (printlevel) 
            fprintf('\n  %s',msg);
            fprintf(': pstep = %3.2e,  dstep = %3.2e\n',pstep,dstep);
         end
         runhist.cputime(iter+1) = cputime-tstart; 
         termcode = -2; 
         breakyes = 1; 
      end
      if (~predcorr)
         if (iter >= 2) 
            idx = [max(2,iter-2) : iter];
            pred_slow = all(mupredhist(idx)./mupredhist(idx-1) > 0.4);
            idx = [max(2,iter-5) : iter];
            pred_convg_rate = mean(mupredhist(idx)./mupredhist(idx-1));
            pred_slow = pred_slow + (mupred/mu > 5*pred_convg_rate);
         end 
         if (max(mu,infeas) < 1e-6) & (pred_slow) & (stoplevel)
            msg = 'sqlp stop: lack of progress in predictor'; 
            if (printlevel) 
               fprintf('\n  %s',msg);
               fprintf(': mupred/mu = %3.2f, pred_convg_rate = %3.2f.',...
               mupred/mu,pred_convg_rate);
            end
            runhist.cputime(iter+1) = cputime-tstart; 
            termcode = -2; 
            breakyes = 1;
         else 
            update_iter = 1; 
         end
      end
%%---------------------------------------------------------------
%% corrector step.
%%---------------------------------------------------------------
%%
      if (predcorr) & (~breakyes)
         step_pred = min(pstep,dstep);
         if (mu > 1e-6)
            if (step_pred < 1/sqrt(3)); 
               expon_used = 1; 
            else
               expon_used = max(expon,3*step_pred^2); 
            end
         else 
            expon_used = max(1,min(expon,3*step_pred^2)); 
         end 
         if (nn==0)
             sigma = 0.2; 
         elseif (mupred < 0) 
             sigma = 0.8; 
         else
            sigma = min(1, (mupred/mu)^expon_used);
         end
         sigmu = cell(size(blk,1),1); 
         for p = 1:size(blk,1)
            sigmu{p} = max(sigma*mu, parbarrier{p}'); 
         end	 
         if (vers == 1)
            [dX,dy,dZ] = HKMcorr(blk,At,par,rp,Rd,sigmu,hRd,...
             dX,dZ,coeff,L,X,Z);
         elseif (vers == 2)
            [dX,dy,dZ] = NTcorr(blk,At,par,rp,Rd,sigmu,hRd,...
             dX,dZ,coeff,L,X,Z); 
         end
         if (solve_ok <= 0)
            msg = 'sqlp stop: difficulty in computing corrector directions'; 
            if (printlevel); fprintf('\n  %s',msg); end
            runhist.pinfeas(iter+1) = runhist.pinfeas(iter); 
            runhist.dinfeas(iter+1) = runhist.dinfeas(iter); 
            runhist.relgap(iter+1)  = runhist.relgap(iter); 
            runhist.cputime(iter+1) = cputime-tstart; 
            termcode = -4;
            break; %% do not ues breakyes = 1
         end
         timenew = cputime;
         ttime.corr = ttime.corr + timenew-timeold; timeold = timenew; 
%%
%%-----------------------------------
%% step-lengths for corrector step
%%-----------------------------------
%%
         if (gam == 0) 
            gamused = 0.9 + 0.09*min(pstep,dstep); 
         else
            gamused = gam;
         end            
         Xstep = steplength(blk,X,dX,Xchol,invXchol);
         pstep = min(1,gamused*full(Xstep));
         timenew = cputime;
         ttime.corr_pstep = ttime.corr_pstep + timenew-timeold; timeold = timenew;
         Zstep = steplength(blk,Z,dZ,Zchol,invZchol);
         dstep = min(1,gamused*full(Zstep));
         trXZnew = trXZ + pstep*blktrace(blk,dX,Z,parbarrier) ...
                    + dstep*blktrace(blk,X,dZ,parbarrier)...
                    + pstep*dstep*blktrace(blk,dX,dZ,parbarrier); 
         if (nn > 0); mucorr  = trXZnew/nn; else; mucorr = 1e-16; end
         timenew = cputime;
         ttime.corr_dstep = ttime.corr_dstep + timenew-timeold; timeold = timenew;
%%
%%-----------------------------------------
%%  stopping criteria for corrector step
%%-----------------------------------------
         if (iter >= 2) 
            idx = [max(2,iter-2) : iter];
            corr_slow = all(runhist.gap(idx)./runhist.gap(idx-1) > 0.8); 
            idx = [max(2,iter-5) : iter];
            corr_convg_rate = mean(runhist.gap(idx)./runhist.gap(idx-1));
            corr_slow = corr_slow + (mucorr/mu > max(min(1,5*corr_convg_rate),0.8));
         end 
	 if (max(relgap,infeas) < 1e-6) & (iter > 20) ...
            & (corr_slow > 1) & (stoplevel)
            msg = 'sqlp stop: lack of progress in corrector'; 
   	    if (printlevel) 
               fprintf('\n  %s',msg);
               fprintf(': mucorr/mu = %3.2f, corr_convg_rate = %3.2f',...
               mucorr/mu,corr_convg_rate); 
            end
            runhist.cputime(iter+1) = cputime-tstart; 
            termcode = -1; 
            breakyes = 1;
         else
            update_iter = 1;
         end
      end 
%%---------------------------------------------------------------
%% udpate iterate
%%---------------------------------------------------------------
      indef = [1,1]; 
      if (update_iter)
         for t = 1:5
            [Xchol,indef(1)] = blkcholfun(blk,ops(X,'+',dX,pstep)); 
            timenew = cputime;
            ttime.pchol = ttime.pchol + timenew-timeold; timeold = timenew;
            if (indef(1)); pstep = 0.8*pstep; else; break; end            
         end
	 if (t > 1); pstep = gamused*pstep; end
	 for t = 1:5
            [Zchol,indef(2)] = blkcholfun(blk,ops(Z,'+',dZ,dstep)); 
            timenew = cputime;
            ttime.dchol = ttime.dchol + timenew-timeold; timeold = timenew; 
            if (indef(2)); dstep = 0.8*dstep; else; break; end             
         end
	 if (t > 1); dstep = gamused*dstep; end
         %%-------------------------------------------
         AXtmp = AX + pstep*AXfun(blk,At,par.permA,dX);
         prim_infeasnew = norm(b-AXtmp)/normb2;
         if (relgap < 5*infeas); alpha = 1e2; else; alpha = 1e3; end
         if any(indef)
            if indef(1); msg = 'sqlp stop: X not positive definite'; end
            if indef(2); msg = 'sqlp stop: Z not positive definite'; end
            if (printlevel); fprintf('\n  %s',msg); end
            termcode = -3;
            breakyes = 1;         
         elseif (prim_infeasnew > max([1e-8,relgap,20*prim_infeas]) & iter > 10) ...
            | (prim_infeasnew > max([1e-7,1e3*prim_infeas,0.1*relgap]) & relgap < 1e-2) ...
            | (prim_infeasnew > alpha*max([1e-9,param.prim_infeas_min]) ...
               & (prim_infeasnew > max([3*prim_infeas,0.1*relgap])) ...
               & (iter > 25) & (dual_infeas < 1e-6) & (relgap < 0.1)) ...
            | ((prim_infeasnew > 1e3*prim_infeas & prim_infeasnew > 1e-12) ...
               & (max(relgap,dual_infeas) < 1e-8))
            if (stoplevel) 
               msg = 'sqlp stop: primal infeas has deteriorated too much'; 
               if (printlevel); fprintf('\n  %s, %2.1e',msg,prim_infeasnew); end
               termcode = -7; 
               breakyes = 1; 
            end
         elseif (trXZnew > 1.05*runhist.gap(iter)) & (~exist_analytic_term) ...
	    & ((infeas < 1e-5) & (relgap < 1e-4) & (iter > 20) ...
	       | (max(infeas,relgap) < 1e-7) & (iter > 10)) 
            if (stoplevel) 
               msg = 'sqlp stop: progress in duality gap has deteriorated'; 
               if (printlevel); fprintf('\n  %s, %2.1e',msg,trXZnew); end
               termcode = -8; 
               breakyes = 1; 
            end
         else
            X = ops(X,'+',dX,pstep);  
            y = y + dstep*dy;           
            Z = ops(Z,'+',dZ,dstep);
         end
      end
%%--------------------------------------------------
%% perturb Z: do this step before checking for break
%%--------------------------------------------------
      if (~breakyes) & (~exist_analytic_term)
         trXZtmp = blktrace(blk,X,Z);
         trXE  = blktrace(blk,X,EE);
         Zpert = max(1e-12,0.2*min(relgap,prim_infeas)).*normC2./normE2;
         Zpert = min(Zpert,0.1*trXZtmp./trXE);
         Zpert = min([1,Zpert,1.5*Zpertold]); 
         if (infeas < 0.1) 
            Z = ops(Z,'+',EE,Zpert); 
            [Zchol,indef(2)] = blkcholfun(blk,Z);
            if any(indef(2))
               msg = 'sqlp stop: Z not positive definite';      
               if (printlevel); fprintf('\n  %s',msg); end
               termcode = -3;
               breakyes = 1; 
            end
            %%if (printlevel > 2); fprintf(' %2.1e',Zpert); end
         end
         Zpertold = Zpert; 
      end
%%---------------------------------------------------------------
%% compute rp, Rd, infeasibities, etc
%%---------------------------------------------------------------
%%
      AX  = AXfun(blk,At,par.permA,X); 
      rp  = b-AX;
      ZpATy = ops(Z,'+',Atyfun(blk,At,par.permA,par.isspAy,y));
      ZpATynorm = ops(ZpATy,'norm');
      Rd  = ops(C,'-',ZpATy);
      objadd = blkbarrier(blk,X,Z,Xchol,Zchol,parbarrier) + objadd0; 
      obj = (normb*normC)*[blktrace(blk,C,X), b'*y] + objadd;  
      gap = (normb*normC)*blktrace(blk,X,Z) - diff(objadd);
      relgap = gap/(1+sum(abs(obj))); 
      prim_infeas = norm(rp)/normb2;
      dual_infeas = ops(Rd,'norm')/normC2;
      infeas = max(prim_infeas,dual_infeas); 
      if (scale_data)
         infeas_org(1) = prim_infeas*normb;
         infeas_org(2) = dual_infeas*normC;
      end
      homRd = inf; homrp = inf; 
      if (ops(parbarrier,'norm') == 0)
         if (obj(2) > 0); homRd = ZpATynorm/(obj(2)); end
         if (obj(1) < 0); homrp = norm(AX)/(-obj(1))/(normC); end
      end
      trXZ = blktrace(blk,X,Z,parbarrier); 
      if (nn > 0); mu = trXZ/nn; else; mu = gap/ops(X,'getM'); end
      normX = ops(X,'norm');
%%
      runhist.pobj(iter+1)  = obj(1); 
      runhist.dobj(iter+1)  = obj(2); 
      runhist.gap(iter+1)   = gap;
      runhist.relgap(iter+1)  = relgap;
      runhist.pinfeas(iter+1) = prim_infeas;
      runhist.dinfeas(iter+1) = dual_infeas;
      runhist.infeas(iter+1)  = infeas;
      runhist.step(iter+1)    = min(pstep,dstep); 
      runhist.normX(iter+1)   = normX; 
      runhist.cputime(iter+1) = cputime-tstart; 
      timenew = cputime;
      ttime.misc = ttime.misc + timenew-timeold; timeold = timenew;  
      [hh,mm,ss] = mytime(sum(runhist.cputime)); 
      if (printlevel>=3)
         fprintf('\n%2.0f|%4.3f|%4.3f',iter,pstep,dstep);
         fprintf('|%2.1e|%2.1e|%2.1e|',prim_infeas,dual_infeas,gap);
         fprintf('%- 7.6e| %s:%s:%s|',mean(obj),hh,mm,ss);
      end
%%--------------------------------------------------
%% check convergence
%%--------------------------------------------------
      param.iter        = iter; 
      param.obj         = obj;
      param.gap         = gap; 
      param.relgap      = relgap; 
      param.prim_infeas = prim_infeas;
      param.dual_infeas = dual_infeas;
      param.mu        = mu; 
      param.homRd     = homRd; 
      param.homrp     = homrp; 
      param.AX        = AX; 
      param.ZpATynorm = ZpATynorm;
      param.normX     = ops(X,'norm'); 
      param.normZ     = ops(Z,'norm'); 
      param.stoplevel = stoplevel; 
      param.termcode  = termcode; 
      param.use_LU    = use_LU; 
      if (~breakyes)
         [param,breakyes,restart,msg2] = sqlpcheckconvg(param,runhist); 
      end
      if (restart)
         [X,y,Z] = infeaspt(blk,At,C,b,2,1e5); 
         rp  = b-AXfun(blk,At,par.permA,X); 
         ZpATy = ops(Z,'+',Atyfun(blk,At,par.permA,par.isspAy,y));
         Rd  = ops(C,'-',ZpATy); 
         trXZ = blktrace(blk,X,Z,parbarrier); 
         mu   = trXZ/nn;
         gap  =  (normb*normC)*blktrace(blk,X,Z) - diff(objadd);
         prim_infeas = norm(rp)/normb2;
         dual_infeas = ops(Rd,'norm')/normC2;
         infeas = max(prim_infeas,dual_infeas); 
         [Xchol,indef(1)] = blkcholfun(blk,X); 
         [Zchol,indef(2)] = blkcholfun(blk,Z); 
         stoplevel = 3;
      end
%%--------------------------------------------------
%% check for break
%%--------------------------------------------------
      if (breakyes); break; end
   end
%%---------------------------------------------------------------
%% end of main loop
%%---------------------------------------------------------------
%%
   use_olditer = 0; 
   if (runhist.pinfeas(iter+1) > 3*runhist.pinfeas(iter)) ...
      & (runhist.relgap(iter+1) > 0.3*runhist.relgap(iter))
      X = Xold;
      Xchol = blkcholfun(blk,X); 
      use_olditer = 1; 
   end
   if (runhist.dinfeas(iter+1) > 3*runhist.dinfeas(iter)) ...
      & (runhist.relgap(iter+1) > 0.3*runhist.relgap(iter))
      Z = Zold; y = yold; 
      Zchol = blkcholfun(blk,Z); 
      use_olditer = 1;
   end
   if (use_olditer)
      AX = AXfun(blk,At,par.permA,X); 
      rp = b-AX;
      ZpATy = ops(Z,'+',Atyfun(blk,At,par.permA,par.isspAy,y));
      Rd = ops(C,'-',ZpATy);
      objadd = blkbarrier(blk,X,Z,Xchol,Zchol,parbarrier) + objadd0; 
      obj = (normb*normC)*[blktrace(blk,C,X), b'*y] + objadd;  
      gap = (normb*normC)*blktrace(blk,X,Z) - diff(objadd);
      relgap = gap/(1+sum(abs(obj)));
      prim_infeas = norm(rp)/normb2; 
      dual_infeas = ops(Rd,'norm')/normC2; 
      infeas = max(prim_infeas,dual_infeas); 
      runhist.pobj(iter+1)  = obj(1); 
      runhist.dobj(iter+1)  = obj(2); 
      runhist.gap(iter+1)   = gap;
      runhist.relgap(iter+1)  = relgap;
      runhist.pinfeas(iter+1) = prim_infeas;
      runhist.dinfeas(iter+1) = dual_infeas;
      runhist.infeas(iter+1)  = infeas; 
   end
%%---------------------------------------------------------------
%% unscale and produce infeasibility certificates if appropriate
%%---------------------------------------------------------------
   if (iter >= 1)
      [X,y,Z,termcode,resid,reldist,msg3] = ...
      sqlpmisc(blk,At,C,b,X,y,Z,par.permZ,param); 
   end
%%---------------------------------------------------------------
%% recover unrestricted blk from qblk
%%---------------------------------------------------------------
%% 
   if (any(u2lblk))
      X = X(1:end-1); Z = Z(1:end-1); 
      y = y(1:length(b)-4); 
      for p = 1:size(blk,1)
         if (u2lblk(p) == 1)
            X{p} = X{p}(2:end-3); 
            Z{p} = Z{p}(2:end-3); 
         end
      end
   end
   for p = 1:size(ublkidx,1) 
      if ~isempty(ublkidx{p,2})
         n0 = ublkidx{p,1}; idxB = setdiff([1:n0]',ublkidx{p,2});
         tmp = zeros(n0,1); tmp(idxB) = X{p}; X{p} = tmp; 
         tmp = zeros(n0,1); tmp(idxB) = Z{p}; Z{p} = tmp; 
      end
   end
   if (analytic_prob)
      X = X(1:numblkold); Z = Z(1:numblkold); 
   end
%%---------------------------------------------------------------
%% print summary
%%---------------------------------------------------------------
%%
   maxC = 1+ops(ops(C,'abs'),'max'); 
   maxb = 1+max(abs(b)); 
   if (scale_data)
      dimacs = [infeas_org(1)*normb2/maxb; 0; infeas_org(2)*normC2/maxC; 0]; 
   else
      dimacs = [prim_infeas*normb2/maxb; 0; dual_infeas*normC2/maxC; 0];
   end
   dimacs = [dimacs; [-diff(obj); gap]/(1+sum(abs(obj)))];
   info.dimacs   = dimacs; 
   info.termcode = termcode;
   info.iter     = iter; 
   info.obj      = obj; 
   info.gap      = gap; 
   info.relgap   = relgap;
   info.pinfeas  = prim_infeas;
   info.dinfeas  = dual_infeas;
   info.cputime  = sum(runhist.cputime); 
   info.ttime    = ttime; 
   info.resid    = resid;
   info.reldist  = reldist; 
   info.normX    = ops(X,'norm'); 
   info.normy    = norm(y); 
   info.normZ    = ops(Z,'norm'); 
   info.normb    = normb2; info.maxb = maxb; 
   info.normC    = normC2; info.maxC = maxC; 
   info.normA    = normA2;
   info.msg1     = msg; 
   info.msg2     = msg2;
   info.msg3     = msg3;
   sqlpsummary(info,ttime,infeas_org,printlevel);
   rand('state',randstate);
   randn('state',randnstate);
%%*****************************************************************************
