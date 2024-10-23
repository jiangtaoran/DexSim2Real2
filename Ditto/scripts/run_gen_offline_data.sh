# python _data_generation/gen_offline_data.py \
#   --data_dir data/drawer/train/scenes \
#   --stereo_data_dir data/drawer_stereo/train/scenes \
#   --data_fn assets/stats/drawer_train.txt \
#   --category_types Drawer \
#   --num_processes 16 \
#   --ins_cnt_fn assets/stats/ins_cnt_drawer.txt\

# python _data_generation/gen_offline_data.py \
#   --data_dir data/drawer/val/scenes \
#   --stereo_data_dir data/drawer_stereo/val/scenes \
#   --data_fn assets/stats/drawer_val.txt \
#   --category_types Drawer \
#   --num_processes 12 \
#   --ins_cnt_fn assets/stats/ins_cnt_drawer.txt \

# python _data_generation/gen_offline_data.py \
#   --data_dir data/faucet/train/scenes \
#   --stereo_data_dir data/faucet_stereo/train/scenes \
#   --data_fn assets/stats/faucet_train.txt \
#   --category_types Faucet \
#   --num_processes 12 \
#   --ins_cnt_fn assets/stats/ins_cnt_faucet.txt \

# python _data_generation/gen_offline_data.py \
#   --data_dir data/faucet/val/scenes \
#   --stereo_data_dir data/faucet_stereo/val/scenes \
#   --data_fn assets/stats/faucet_val.txt \
#   --category_types Faucet \
#   --num_processes 8 \
#   --ins_cnt_fn assets/stats/ins_cnt_faucet.txt \

 python data_generation_allegro/gen_offline_data.py \
   --data_dir data/drawer_allegro/train/scenes \
   --stereo_data_dir data/drawer_stereo_allegro/train/scenes \
   --data_fn assets/stats/drawer_train.txt \
   --category_types Drawer \
   --num_processes 8 \
   --ins_cnt_fn assets/stats/ins_cnt_drawer.txt

# python data_generation_allegro/gen_offline_data.py \
#   --data_dir data/drawer_allegro2/val/scenes \
#   --stereo_data_dir data/drawer_stereo_allegro2/val/scenes \
#   --data_fn assets/stats/drawer_val.txt \
#   --category_types Drawer \
#   --num_processes 8 \
#   --ins_cnt_fn assets/stats/ins_cnt_drawer.txt