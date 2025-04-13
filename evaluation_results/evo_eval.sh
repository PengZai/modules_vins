
DATASET="BotanicGarden"
DIR="1018_00"
TEST_DIR="test_0"
ESITMATED_FOLDER="modules_vins_stereo_for_1018_00_img10hz600p"
REFERENCE="${DIR}_GT_output.txt"

evo_rpe tum $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/modules_vins_stereo_for_1018_00_img10hz600p_estimated_traj_coordinate_aligned.txt $DATASET/$DIR/$REFERENCE --plot_mode=xy --save_plot ${DATASET}/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/2d_rpe_traj_estimate > $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/rpe_estimated_traj.log
evo_ape tum $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/modules_vins_stereo_for_1018_00_img10hz600p_estimated_traj_coordinate_aligned.txt $DATASET/$DIR/$REFERENCE --plot_mode=xy --save_plot ${DATASET}/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/2d_ape_traj_estimate > $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/ape_estimated_traj.log
evo_ape tum $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/modules_vins_stereo_for_1018_00_img10hz600p_estimated_traj_coordinate_aligned.txt $DATASET/$DIR/$REFERENCE --align --pose_relation trans_part --plot_mode=xy --save_plot ${DATASET}/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/2d_ate_traj_estimate > $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/ate_estimated_traj.log


evo_traj tum $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/modules_vins_stereo_for_1018_00_img10hz600p_estimated_traj_coordinate_aligned.txt --ref=$DATASET/$DIR/$REFERENCE -p --plot_mode=xyz --save_plot $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/3d  
evo_traj tum $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/modules_vins_stereo_for_1018_00_img10hz600p_estimated_traj_coordinate_aligned.txt --ref=$DATASET/$DIR/$REFERENCE -p --plot_mode=xy --save_plot $DATASET/$DIR/${ESITMATED_FOLDER}/${TEST_DIR}/2d 
