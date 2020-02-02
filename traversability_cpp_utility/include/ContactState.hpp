#ifndef CONTACT_STATE_H
#define CONTACT_STATE_H

class ContactState
{
public:
    ContactState(RPYTF _left_foot_pose, RPYTF _right_foot_pose, RPYTF _left_hand_pose,
         RPYTF _right_hand_pose, std::shared_ptr<ContactState> _parent, float _g, float _h, ContactManipulator _move_manip):
         left_foot_pose_(_left_foot_pose),
         right_foot_pose_(_right_foot_pose),
         left_hand_pose_(_left_hand_pose),
         right_hand_pose_(_right_hand_pose),
         parent_(_parent),
         g_(_g),
         h_(_h),
         prev_move_manip_(_move_manip)
         {};

    // foot orientation projected to flat gruond
    // float get_left_horizontal_yaw() const;
    // float get_right_horizontal_yaw() const;

    const RPYTF left_foot_pose_;
    const RPYTF right_foot_pose_;
    const RPYTF left_hand_pose_;
    const RPYTF right_hand_pose_;

    const ContactManipulator prev_move_manip_;
    const float g_;
    const float h_;

    int explore_state_;
    
    bool left_arm_moved_ = false;
    bool right_arm_moved_ = false;

    inline bool operator==(const ContactState& other) const{ return ((this->left_foot_pose_ == other.left_foot_pose_) &&
                                                                     (this->right_foot_pose_ == other.right_foot_pose_) &&
                                                                     (this->left_hand_pose_ == other.left_hand_pose_) &&
                                                                     (this->right_hand_pose_ == other.right_hand_pose_));}

    inline bool operator!=(const ContactState& other) const{ return ((this->left_foot_pose_ != other.left_foot_pose_) ||
                                                                     (this->right_foot_pose_ != other.right_foot_pose_) ||
                                                                     (this->left_hand_pose_ != other.left_hand_pose_) ||
																	 (this->right_hand_pose_ != other.right_hand_pose_));}
																	 
	inline bool operator<(const ContactState& other) const { return (this->getF() < other.getF());}


	inline std::shared_ptr<ContactState>  getParent() {return parent_;}
	inline const float getF() const {return (g_+h_);}

private:
    std::shared_ptr<ContactState> parent_;
    // float edge_cost_;
    // float dr_ability_; // disturbance rejection ability
};

#endif