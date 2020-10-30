import os,pickle
import subprocess
import multiprocessing

def worker(cmd):
    FNULL=open(os.devnull,'w')
    subprocess.call(cmd,stdin=FNULL,stdout=FNULL,stderr=FNULL,shell=False)
    return None

def render_animation(folder,dur=1.,parallel=True):
    if not os.path.exists(folder):
        return
    cmds=None
    if not os.path.exists(folder+'/cmd.dat'):
        cmds=[]
        import re
        for f in os.listdir(folder):
            if re.match('[0-9]+.pov',f): 
                cmds.append(('povray',folder+"/"+f))
    
    #render  
    frms=[]
    cmds_parallel=[]
    if cmds is None:
        cmds=pickle.load(open(folder+'/cmd.dat','rb'))  
    for cmd in cmds:
        out=cmd[1][0:len(cmd[1])-4]+'.png'
        for it,t in enumerate(cmd):
            if t.startswith('+O'):
                cmd[it]='+O%s'%out
            elif t.startswith('Output_File_Type'):
                cmd[it]='Output_File_Type=N'
        print('Executing: ',cmd)
        if not os.path.exists(out):
            if not parallel:    #single processing
                process=subprocess.Popen(cmd,stderr=subprocess.PIPE,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
                out,err=process.communicate('render_animation'.encode('ascii'))
            else: cmds_parallel.append(cmd)
        frms.append(out)
        
    #multiprocessing
    if parallel:
        nrCPU=multiprocessing.cpu_count()//2
        print("Using %d CPUs!"%nrCPU)
        
        nrFinished=0
        batched=False
        pool=multiprocessing.Pool(nrCPU)
        if batched:
            while len(cmds_parallel)>0:
                nrSubmit=min(nrCPU,len(cmds_parallel))
                pool.map(worker,cmds_parallel[0:nrSubmit])
                cmds_parallel=cmds_parallel[nrSubmit:]
                #profile
                nrFinished+=nrSubmit
                print("Finished %d/%d"%(nrFinished,len(cmds_parallel)))
        else: pool.map(worker,cmds_parallel)
        pool.close()
        pool.join()
            
    #output animation
    if not os.path.exists(folder+'/vid.AVI'):
        import cv2
        height,width,layers=cv2.imread(frms[0]).shape
        out=cv2.VideoWriter(folder+'/vid.AVI',cv2.VideoWriter_fourcc(*'DIVX'),1./dur,(width,height))
        for id,f in enumerate(frms):
            print("Rendering %d/%d video frame!"%(id,len(frms)))
            out.write(cv2.imread(f))
        out.release()
                
if __name__=='__main__':
    render_animation('tmpAnimation',dur=.1)
        