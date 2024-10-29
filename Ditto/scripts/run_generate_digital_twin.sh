#for i in 1 2 3 4 5 6 7 8
#do
#  python real_experiment/generate_digital_twin.py $i
#done
#python real_experiment/generate_digital_twin.py drawer 7
#python real_experiment/generate_digital_twin.py drawer 8

object_name=$1
cnt_id=$2
ckpt_path=$3

python real_experiment/generate_digital_twin.py $object_name $cnt_id $ckpt_path
cd real_test/real_datasets_allegro/$object_name/video_${cnt_id}/digital_twin/${object_name}_video_${cnt_id}
cp ../../../../../TestVHACD .
./TestVHACD mesh_0.obj
mv decomp.obj mesh_0_decomp.obj
./TestVHACD mesh_1.obj
mv decomp.obj mesh_1_decomp.obj
rm TestVHACD
rm decomp*

#python real_experiment/generate_digital_twin.py faucet 1
#cd /home/guest2/Documents/Sim2Real2/ditto/real_test/real_datasets/faucet/video_1/digital_twin/faucet_video_1
#./TestVHACD mesh_0.obj
#mv decomp.obj mesh_0_decomp.obj
#./TestVHACD mesh_1.obj
#mv decomp.obj mesh_1_decomp.obj