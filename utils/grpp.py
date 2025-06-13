import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('../')
from utils.util import soft_squash_multidim,kernel_levse

class GaussianRandomPathClass(object):
    def __init__(self,
                 name     = 'GRP',
                 kernel   = kernel_levse,
                 hyp_mean = {'g':1.0,'l':1.0,'w':1e-6},
                 hyp_var  = {'g':1.0,'l':1.0,'w':1e-6}
                 ):
        # super(GaussianRandomPathClass,self).__init__()
        self.name = name
        # Set kernel
        self.kernel   = kernel     # kernel function
        self.hyp_mean = hyp_mean
        self.hyp_var  = hyp_var
        # Default set data
        self.set_data()
        
    def set_data(self,
                 t_anchor     = np.linspace(start=0.0,stop=1.0,num=10).reshape((-1,1)),
                 x_anchor     = np.zeros((10,2)),
                 l_anchor     = np.ones((10,1)),
                 t_test       = np.linspace(start=0.0,stop=1.0,num=100).reshape((-1,1)),
                 l_test       = np.ones((100,1)),
                 hyp_mean     = None,
                 hyp_var      = None,
                 w_chol       = 1e-10,     # noise for Cholesky transform
                 APPLY_EPSRU  = False,     # epsilon run-up
                 t_eps        = 0.0001,
                 l_eps        = 1.0,       # leverage for epsilon run-up
                 x_diff_start = None,
                 x_diff_end   = None,
                 SKIP_GP_VAR  = False      # skip GP variance computation
                 ):
        """
            Set anchor data to GRP class
        """
        self.t_anchor    = t_anchor.astype(float)    # [N x 1]
        self.x_anchor    = x_anchor.astype(float)    # [N x D]
        self.t_test      = t_test.astype(float)      # [N_test x 1]
        self.n_anchor    = self.x_anchor.shape[0]
        self.d_anchor    = self.x_anchor.shape[1]
        self.l_anchor    = l_anchor.astype(float)    # [N x 1]
        self.n_test      = self.t_test.shape[0]
        self.l_test      = l_test.astype(float)      # [N_test x 1]
        if hyp_mean is not None: self.hyp_mean = hyp_mean
        if hyp_var is not None: self.hyp_var = hyp_var
            
        # Handle epsilon greedy algorithm
        self.APPLY_EPSRU = APPLY_EPSRU
        self.t_eps       = t_eps
        if self.APPLY_EPSRU:
            # Append runup points
            if (x_diff_start is not None) and (x_diff_end is not None):
                self.x_diff_start = x_diff_start
                self.x_diff_end   = x_diff_end
            else:
                self.x_diff_start = (self.x_anchor[-1,:]-self.x_anchor[0,:]).reshape((1,-1))
                self.x_diff_end = (self.x_anchor[-1,:]-self.x_anchor[0,:]).reshape((1,-1))
            self.t_dur  = (self.t_anchor[-1]-self.t_anchor[0]).squeeze()
            # Append 'x_anchor'
            x_anchor    = self.x_anchor
            x_anchor    = np.insert(x_anchor,1,
                                    x_anchor[0,:]+self.t_eps/self.t_dur*self.x_diff_start,axis=0)
            x_anchor    = np.insert(x_anchor,-1,
                                    x_anchor[-1,:]-self.t_eps/self.t_dur*self.x_diff_end,axis=0)
            n_anchor    = self.x_anchor.shape[0]
            # Append 'x_anchor'
            t_anchor    = self.t_anchor
            t_anchor    = np.insert(t_anchor,1,t_anchor[0,:]+self.t_eps,axis=0)
            t_anchor    = np.insert(t_anchor,-1,t_anchor[-1,:]-self.t_eps,axis=0)
            # Append 'l_anchor'
            l_anchor    = self.l_anchor
            l_eps       = 0.0
            l_anchor    = np.insert(l_anchor,1,l_eps,axis=0)
            l_anchor    = np.insert(l_anchor,-1,l_eps,axis=0)
            # Overwrite 'x_anchor', 't_anchor', 'l_anchor', and 'n_anchor'
            self.x_anchor = x_anchor
            self.t_anchor = t_anchor
            self.n_anchor = self.x_anchor.shape[0]
            if self.kernel.__name__ == 'kernel_levse': # leveraged SE kernel
                self.l_anchor = l_anchor
                
        # GP mean-related
        if self.kernel.__name__ == 'kernel_levse': # leveraged SE kernel
            l_anchor_mean = np.ones((self.n_anchor,1)) # leverage does not affect GP mean
            self.k_test_anchor_mean   = self.kernel(self.t_test,self.t_anchor,
                                                    self.l_test,l_anchor_mean,
                                                    self.hyp_mean)
            self.K_anchor_anchor_mean = self.kernel(self.t_anchor,self.t_anchor,
                                                    l_anchor_mean,l_anchor_mean,
                                                    self.hyp_mean) \
                                        + self.hyp_mean['w']*np.eye(self.n_anchor)
        elif self.kernel.__name__ == 'kernel_se': # SE kernel
            self.k_test_anchor_mean   = self.kernel(self.t_test,self.t_anchor,
                                                    self.hyp_mean)
            self.K_anchor_anchor_mean = self.kernel(self.t_anchor,self.t_anchor,
                                                    self.hyp_mean) \
                                        + self.hyp_mean['w']*np.eye(self.n_anchor)
        else:
            raise TypeError("[GaussianRandomPathClass] Unsupported kernel:[%s]"%
                            (self.kernel.__name__))
        self.x_anchor_mean        = self.x_anchor.mean(axis=0)
        self.gamma_test           = np.linalg.solve(self.K_anchor_anchor_mean,
                                                    self.x_anchor-self.x_anchor_mean)
        self.mean_test            = np.matmul(self.k_test_anchor_mean,self.gamma_test) \
                                    + self.x_anchor_mean
        
        # GP variance-related
        self.SKIP_GP_VAR = SKIP_GP_VAR
        if self.SKIP_GP_VAR: return # skip in case of computing the mean only 
        if self.kernel.__name__ == 'kernel_levse': # leveraged SE kernel
            self.k_test_test_var     = self.kernel(self.t_test,self.t_test,
                                                   self.l_test,self.l_test,
                                                   self.hyp_var)
            self.k_test_anchor_var   = self.kernel(self.t_test,self.t_anchor,
                                                   self.l_test,self.l_anchor,
                                                   self.hyp_var)
            self.K_anchor_anchor_var = self.kernel(self.t_anchor,self.t_anchor,
                                                   self.l_anchor,self.l_anchor,
                                                   self.hyp_var) \
                                        + self.hyp_mean['w']*np.eye(self.n_anchor)
        elif self.kernel.__name__ == 'kernel_se': # SE kernel
            self.k_test_test_var     = self.kernel(self.t_test,self.t_test,
                                                   self.hyp_var)
            self.k_test_anchor_var   = self.kernel(self.t_test,self.t_anchor,
                                                   self.hyp_var)
            self.K_anchor_anchor_var = self.kernel(self.t_anchor,self.t_anchor,
                                                   self.hyp_var) \
                                        + self.hyp_mean['w']*np.eye(self.n_anchor)
        else:
            raise TypeError("[GaussianRandomPathClass] Unsupported kernel:[%s]"%
                            (self.kernel.__name__))
        self.var_test            = self.k_test_test_var - np.matmul(self.k_test_anchor_var,
            np.linalg.solve(self.K_anchor_anchor_var,self.k_test_anchor_var.T))
        self.var_diag_test       = np.diag(self.var_test).reshape((-1,1))
        self.std_diag_test       = np.sqrt(self.var_diag_test)
        self.w_chol              = w_chol
        self.var_chol_test       = np.linalg.cholesky(self.var_test \
                                                      + self.w_chol*np.eye(self.n_test))
            
    def sample(self,
               n_sample  = 10,
               rand_type = 'Gaussian'):
        """
            Sample from GRP
        """
        samples = []
        for s_idx in range(n_sample):
            if rand_type == 'Gaussian':
                R = np.random.randn(self.n_test,self.d_anchor)
            elif rand_type == 'Uniform':
                rand_gain = 2 # -gain ~ +gain
                R = rand_gain*(2*np.random.rand(self.n_test,self.d_anchor)-1)
            else:
                raise TypeError("[GaussianRandomPathClass] Unsupported rand_type:[%s]"%(rand_type))
            sample = self.mean_test+np.matmul(self.var_chol_test,R)
            samples.append(sample)
        return samples,self.t_test

    def sample_one_traj(
        self,rand_type='Gaussian',ORG_PERTURB=False,perturb_gain=1.0,
        ss_x_min=None,ss_x_max=None,ss_margin=None
    ):
        """
            Sample a single trajectory
        """
        if rand_type == 'Gaussian':
            R = np.random.randn(self.n_test,self.d_anchor)
        elif rand_type == 'Uniform':
            rand_gain = 2 # -gain ~ +gain
            R = rand_gain*(2*np.random.rand(self.n_test,self.d_anchor)-1)
        else:
            raise TypeError("[GaussianRandomPathClass] Unsupported rand_type:[%s]"%(rand_type))
        sample_traj = self.mean_test+np.matmul(self.var_chol_test,R) # [L x dim]
        if ORG_PERTURB:
            L = sample_traj.shape[0]
            D = sample_traj.shape[1]
            pvec = 2.0*perturb_gain*np.random.rand(1,D)-perturb_gain
            sample_traj = sample_traj + np.tile(pvec,(L,1))
        # (optional) soft-squash
        if (ss_x_min is not None) and (ss_x_max is not None) and (ss_margin is not None):
            sample_traj = soft_squash_multidim(
                x=sample_traj,x_min=ss_x_min,x_max=ss_x_max,margin=ss_margin)
        return sample_traj,self.t_test
    
    def plot(self,
             n_sample   = 10,
             ss_x_min   = None,
             ss_x_max   = None,
             ss_margin  = None,
             figsize    = (6,3),
             subplot_rc = None,
             lw_sample  = 1/4,
             ylim       = None,
             title_str  = None,
             tfs        = 10,
             rand_type  = 'Uniform',
             ):
        """
            Plot GRP
        """
        sampled_trajs,t_test = self.sample(n_sample=n_sample,rand_type=rand_type)
        colors = [plt.cm.Set1(i) for i in range(self.d_anchor)]
        if subplot_rc is not None:
            fig = plt.figure(figsize=figsize)
        for d_idx in range(self.d_anchor):
            color = colors[d_idx]
            if subplot_rc is None:
                plt.figure(figsize=figsize)
            else:
                plt.subplot(subplot_rc[0],subplot_rc[1],d_idx+1)
                fig.tight_layout()
            # Plot sampled trajectories
            for s_idx in range(len(sampled_trajs)):
                sampled_traj = sampled_trajs[s_idx]
                plt.plot(self.t_test,sampled_traj[:,d_idx],'-',color='k',lw=lw_sample)
            # Plot squashed trajectories
            if (ss_x_min is not None) and (ss_x_max is not None) and (ss_margin is not None):
                for s_idx in range(len(sampled_trajs)):
                    sampled_traj = sampled_trajs[s_idx]
                    squased_traj = soft_squash_multidim(
                        x      = sampled_traj,
                        x_min  = ss_x_min,
                        x_max  = ss_x_max,
                        margin = ss_margin
                        )
                    plt.plot(self.t_test,squased_traj[:,d_idx],'--',color='k',lw=lw_sample*2)
            # Plot mean
            plt.plot(self.t_test,self.mean_test[:,d_idx],'-',color=color,lw=3)
            # Plot anchors
            plt.plot(self.t_anchor,self.x_anchor[:,d_idx],'o',mfc='none',ms=10,mec='k')
            # Plot 2-standard deviation (95%)
            plt.fill_between(self.t_test.squeeze(),
                             self.mean_test[:,d_idx]-2*self.std_diag_test.squeeze(),
                             self.mean_test[:,d_idx]+2*self.std_diag_test.squeeze(),
                             facecolor=color,interpolate=True,alpha=0.2)
            plt.xlim([self.t_test[0],self.t_test[-1]])
            if ylim is not None:
                plt.ylim(ylim)
            if title_str is None:
                plt.title("Dim-[%d]"%(d_idx),fontsize=tfs)
            else:
                plt.title("%s"%(title_str),fontsize=tfs)
            if subplot_rc is None: plt.show()
        if subplot_rc is not None: plt.show()
    
    def set_prior(
        self,
        q_init  = np.zeros(8),
        dur_sec = 2.0,
        HZ      = 50,
        hyp     = {'g':1.5,'l':1/2,'w':1e-8},
        eps_sec = 0.01):
        """
            Set GRP prior
        """
        # Eps-runup
        if eps_sec == 0:
            t_anchor = np.array([[0.0,dur_sec]]).T
            x_anchor = np.tile(q_init,reps=(2,1))
            l_anchor = np.array([[1,1]]).T
        else:
            t_anchor = np.array([[0.0,eps_sec,dur_sec-eps_sec,dur_sec]]).T
            x_anchor = np.tile(q_init,reps=(4,1))
            l_anchor = np.array([[1,1,1,1]]).T
        n_test   = int(dur_sec*HZ)
        t_test   = np.linspace(start=0.0,stop=dur_sec,num=n_test).reshape((-1,1))
        l_test   = np.ones((n_test,1))
        self.set_data(
            t_anchor=t_anchor,x_anchor=x_anchor,l_anchor=l_anchor,
            t_test=t_test,l_test=l_test,hyp_mean=hyp,hyp_var=hyp,w_chol=1e-10)

    def interpolate(
        self,t_anchor,x_anchor,t_test,hyp={'g':1,'l':1/5,'w':1e-8},
        APPLY_EPSRU=False,t_eps=0.05,x_diff_start=None,x_diff_end=None,
        x_min=None,x_max=None,margin=1.0):
        """
            Interpolation
        """
        n_anchor,n_test = t_anchor.shape[0],t_test.shape[0]
        l_anchor,l_test = np.ones(shape=(n_anchor,1)),np.ones(shape=(n_test,1))
        self.set_data(
            t_anchor=t_anchor,x_anchor=x_anchor,l_anchor=l_anchor,t_test=t_test,l_test=l_test,hyp_mean=hyp,hyp_var=hyp,
            APPLY_EPSRU=APPLY_EPSRU,t_eps=t_eps,x_diff_start=x_diff_start,x_diff_end=x_diff_end,SKIP_GP_VAR=True)
        traj_test = self.mean_test
        if (x_min is not None) and (x_max is not None):
            traj_test = soft_squash_multidim(x=traj_test,x_min=x_min,x_max=x_max,margin=margin)
        return traj_test

    def lev_interpolate(
        self,t_anchor,x_anchor,t_test,hyp={'g':1,'l':1/5,'w':1e-8},lev_val=0.9,
        APPLY_EPSRU=False,t_eps=0.05,x_diff_start=None,x_diff_end=None,
        x_min=None,x_max=None,margin=1.0):
        """
            Leveraged interpolation
        """
        n_anchor,n_test = t_anchor.shape[0],t_test.shape[0]
        l_anchor,l_test = lev_val*np.ones(shape=(n_anchor,1)),np.ones(shape=(n_test,1))
        self.set_data(
            t_anchor=t_anchor,x_anchor=x_anchor,l_anchor=l_anchor,t_test=t_test,l_test=l_test,hyp_mean=hyp,hyp_var=hyp,
            APPLY_EPSRU=APPLY_EPSRU,t_eps=t_eps,x_diff_start=x_diff_start,x_diff_end=x_diff_end,SKIP_GP_VAR=False)
        traj_sample,_ = self.sample_one_traj(rand_type='Uniform')
        if (x_min is not None) and (x_max is not None):
            traj_sample = soft_squash_multidim(x=traj_sample,x_min=x_min,x_max=x_max,margin=margin)
        return traj_sample
