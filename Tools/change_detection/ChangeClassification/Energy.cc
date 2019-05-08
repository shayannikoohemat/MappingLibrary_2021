#include "class_par.h"

//local continuous energy
double ContextEnergy(LaserPoints &lpts,LaserPoints::iterator centerpt)
{
    float                       belta=0.05,energy=0.0;
    LaserPoints::iterator        lpt;
    
    for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
    {
       if(lpt->Attribute(LabelTag)==centerpt->Attribute(LabelTag))
       energy=+belta;
       else energy=-belta;
    }
    return energy;
}


double Gaussian(LaserPoints &lpts, LaserPoints::iterator centerpt)
{
    float Average(LaserPoints &, LaserPointTag);
    float                        var_vz,var_slp,covariance=0.0,mean_vz=0.0,mean_slp=0.0,sigma,incov[2],gauss,gaussion;
    LaserPoints::iterator         lpt;                                                 
       
    mean_vz=Average(lpts,v_zTag);
    mean_slp=Average(lpts,ResidualTag);
    
    for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
     {
        float slp=lpt->FloatAttribute(ResidualTag)-mean_slp;
        float vz=lpt->FloatAttribute(v_zTag)- mean_vz;
        covariance=covariance + slp*vz; 
        var_vz=var_vz+vz*vz;
        var_slp=var_slp+slp*slp;
     }
     covariance=sqrt(covariance/(lpts.size()-1));
     var_vz=sqrt(var_vz/(lpts.size()-1));
     var_slp=sqrt(var_slp/(lpts.size()-1));
     //inverse covariance matrix
     sigma=var_vz*var_slp-covariance*covariance;
     incov[0]=var_vz;
     incov[1]=covariance;
     incov[2]=covariance;
     incov[3]= var_slp;
     gauss=(centerpt->FloatAttribute(v_zTag)-mean_vz)*incov[0]+(centerpt->FloatAttribute(ResidualTag)-mean_slp)*incov[2]*(centerpt->FloatAttribute(v_zTag)-mean_vz)
     +(centerpt->FloatAttribute(v_zTag)-mean_vz)*incov[1]+(centerpt->FloatAttribute(ResidualTag)-mean_slp)*incov[3]*(centerpt->FloatAttribute(ResidualTag)-mean_slp);
     //gaussian energy
     gaussion=(1/(sqrt(2.0*3.141592653589793)*sigma)) * exp((-1*gauss)/(2*sigma*sigma));     
     if(gaussion>=0.6) return gaussion;
     else return -1.0*gaussion;
}

float LocalEnergy(LaserPoints &pts, LaserPoints::iterator mypoint)
{
     return ContextEnergy(pts,mypoint)+Gaussian(pts,mypoint);
}

float GlobalEnergy(LaserPoints &lpts)
{
    LaserPoints::iterator         lpt;
    float                         sum=0.0;
    
    for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
    {
          if(lpt->HasAttribute(v_xTag))sum=sum+lpt->FloatAttribute(v_xTag);
          cout<<"sum:"<<sum<<endl;
    }
    return sum;
}
