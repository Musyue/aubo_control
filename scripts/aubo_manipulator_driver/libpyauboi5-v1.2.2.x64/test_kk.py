import numpy
import math
def transfer_2normal(a,b,c,trans_point):
    theta=math.atan(c/a)
    pha=math.acos(-b/math.sqrt(a*a+b*b+c*c))
    Rxph=[1,0,0,0,math.cos(pha),-math.sin(pha),0,math.sin(pha),math.cos(pha)]
    Rzthe=[math.cos(theta),-math.sin(theta),0,math.sin(theta),math.cos(theta),0,0,0,1]
    nRc=numpy.dot(numpy.matrix(Rxph).reshape((3,3)),numpy.matrix(Rxph).reshape((3,3)))
    print(Rxph)
    print(Rzthe)
    print(nRc)
    nRc_list=nRc.tolist()
    nRctemp=[]
    reslist=[]
    for i in range(len(nRc_list)):
        for j in range(len(nRc_list[i])):
            nRctemp.append(nRc_list[i][j])
    for i in range(len(nRctemp)):
        if i ==2:
            reslist.append(nRctemp[i])
            reslist.append(trans_point[0])
        elif i==5:
            reslist.append(nRctemp[i])
            reslist.append(trans_point[1])
        elif i==8:
            reslist.append(nRctemp[i])
            reslist.append(trans_point[2])
        else:
            reslist.append(nRctemp[i])

    return reslist+[0,0,0,1]
#-0.1635228,-0.1764399,0.9527013,0.1015114,-0.0001705292,-1,0.9624333
#0.04474885,0.009695175,-1,0.9503967
trans_data=[-0.1635228,-0.1764399,0.9527013]
print(transfer_2normal(0.04474885,0.009695175,-1,trans_data))