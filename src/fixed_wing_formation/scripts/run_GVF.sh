uav_id=0
while(( $uav_id< $2 )) 
do
    python GVF_ode.py $1 $uav_id $2 &
    let "uav_id++"
done
