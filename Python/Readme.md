# Simple Follower

Read the MT6701 magnetic encoders and send values to the Feetech STS3215 servos of a follower arm.

conda create -y -n encoder_arm python=3.10
conda activate encoder_arm
pip install feetech-servo-sdk