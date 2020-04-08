#define			CODE_Cmd_NOP                                            0x00			//
#define			CODE_Cmd_DEPLOY_BLOCK                                   0x01			//�����ł��������u���b�N�̃u���b�N�R�}���h��W�J���ă^�C�����C���R�}���h�Ɋi�[����
#define			CODE_Cmd_CLEAR_BLOCK                                    0x02			//�����ŗ^�����u���b�N�̃u���b�N�R�}���h���N���A����
#define			CODE_Cmd_SET_BLOCK_POSITION                             0x03			//�u���b�N�R�}���h�̓o�^�ʒu���w�肷��B
#define			CODE_Cmd_ACTIVATE_BLOCK                                 0x04			//�u���b�N�R�}���h��L��������
#define			CODE_Cmd_FINISH_TRANSITION                              0x05			//���[�h�J�ڏI���g���K�R�}���h
#define			CODE_Cmd_SET_TRANSITION_TABLE                           0x06			//TransitionTable���������R�}���h(from, to , index)
#define			CODE_Cmd_EXECUTE_APP                                    0x07			//�A�v���P�[�V�������s�R�}���h
#define			CODE_Cmd_TDSP_SET_TASK_LIST_ID                          0x08			//�^�X�N���X�gBC�ݒ�R�}���h
#define			CODE_Cmd_ROTATE_BLOCK                                   0x09			//BC��]�R�}���h
#define			CODE_Cmd_COMBINE_BLOCK                                  0x0A			//BC�Z���R�}���h
#define			CODE_Cmd_AH_register_rule                               0x0B			//�A�m�}�����[���ݒ�R�}���h(id, group)
#define			CODE_Cmd_AH_activate_rule                               0x0C			//�A�m�}�����[���L�����R�}���h
#define			CODE_Cmd_AH_inactivate_rule                             0x0D			//�A�m�}�����[���������R�}���h
#define			CODE_Cmd_AH_clear_log                                   0x0E			//�A�m�}�����O�N���A�R�}���h
#define			CODE_Cmd_AS_set_id                                      0x0F			//�A�v��ID�ݒ�R�}���h
#define			CODE_Cmd_REGISTER_APP                                   0x10			//�A�v���o�^�R�}���h
#define			CODE_Cmd_INITIALIZE_APP                                 0x11			//�A�v���������R�}���h
#define			CODE_Cmd_SET_MODE_LIST                                  0x12			//
#define			CODE_Cmd_START_TRANSITION                               0x13			//�^�p���[�h��ύX����
#define			CODE_Cmd_TlmRequest                                     0x14			//�w��TLM_ID�̃e�����g�����_�E�������N����
#define			CODE_Cmd_TLM_SET_MODE                                   0x15			//OBC�e�����g�����[�h�ݒ�F�J�n
#define			CODE_Cmd_SET_TLM_INTERVAL                               0x16			//interval[s]
#define			CODE_Cmd_CLEAR_ALL_TIMELINE                             0x18			//MOBC�R�}���h�N���A(�^�C�����C��)
#define			CODE_Cmd_CLEAR_TIMELINE_AT                              0x19			//MOBC�ɓo�^����Ă���^�C�����C���R�}���h����1�I��ō폜����
#define			CODE_Cmd_SET_OBC_TIME                                   0x1A			//MOBC�����^�O����͒l�ɐݒ肷��
#define			CODE_Cmd_STRX_SET_BR                                    0x20			//param0: 0=4kbps, 1:32kbps, 2:64kbps (IF���STRX�{�̓����ɕύX)
#define			CODE_Cmd_STRX_TX_ON                                     0x21			//param0: 0=OFF, 1=ON
#define			CODE_Cmd_MUX_UART_SET                                   0x25			//
#define			CODE_Cmd_SYNC_TIME                                      0x26			//�n��ǎ����Ɖq��(OBC, RTC)�����𓯊�������
#define			CODE_Cmd_SYNC_OBC_TIME                                  0x27			//�n��ǎ�����OBC�����𓯊�������
#define			CODE_Cmd_SYNC_RTC_TIME                                  0x28			//�n��ǎ�����RTC�����𓯊�������
#define			CODE_Cmd_SYNC_GPS_TIME                                  0x29			//GPS�����Ɖq�������𓯊�������
#define			CODE_Cmd_FRAM_SYNC_RTC_ENABLE                           0x2A			//OBC���Z�b�g����RTC�Ǝ�������������(ENA:1, DIS:0)
#define			CODE_Cmd_FRAM_TLC_CLEAR_ALL                             0x2B			//OBC-FRAM�̃R�}���h�N���A(�^�C�����C��)
#define			CODE_Cmd_FRAM_TLC_CLEAR_AT_TIME                         0x2C			//OBC-FRAM�ɓo�^����Ă���^�C�����C���R�}���h���玞���w���1�I��ō폜����
#define			CODE_Cmd_FRAM_TLC_CLEAR_AT_NUMBER                       0x2D			//OBC-FRAM�ɓo�^����Ă���^�C�����C���R�}���h����ԍ��w���1�I��ō폜����
#define			CODE_Cmd_SOFTWARE_RESET                                 0x31			//�\�t�g�������Z�b�g
#define			CODE_Cmd_FRAM_INITIALIZE                                0x32			//FRAM�̏�����, �ł��グ�O���[�h�ɂ���
#define			CODE_Cmd_FRAM_CMD_INITIALIZE                            0x33			//FRAM(TLC, BC)������������(�S����)
#define			CODE_Cmd_FRAM_DEPLOY_BLOCK                              0x34			//�����ł��������u���b�N�̃u���b�N�R�}���h��W�J���ă^�C�����C���R�}���h�Ɋi�[����
#define			CODE_Cmd_FRAM_REMOVE_BLOCK                              0x35			//�����ŗ^�����u���b�N�̃u���b�N�R�}���h���N���A����
#define			CODE_Cmd_FRAM_SET_BLOCK_POSITION                        0x36			//�u���b�N�R�}���h�̓o�^�ʒu���w�肷��B
#define			CODE_Cmd_FRAM_ACTIVATE_BLOCK                            0x37			//�u���b�N�R�}���h�̗L���ɂ���
#define			CODE_Cmd_FRAM_REMOVE_BLOCK_AT_POSITION                  0x38			//�w��ID�̈ʒu�̃u���b�N�R�}���h����������
#define			CODE_Cmd_FRAM_REMOVE_ALL_BLOCK                          0x39			//�o�^�ς݂̃u���b�N�R�}���h�̏������ׂď���
#define			CODE_Cmd_FRAM_REQUEST_BC_INFO                           0x3A			//�w��ID�̃u���b�N�R�}���h�����_�E�������N����
#define			CODE_Cmd_FRAM_REQUEST_BC_PARAM                          0x3B			//�w��ID, �ʒu�̃u���b�N�R�}���h�̎��s�p�P�b�g���_�E�������N����
#define			CODE_Cmd_STRX_SET_TX_PWR                                0x40			//Power Mode�̐؂�ւ�
#define			CODE_Cmd_STRX_OTC_ENABLE                                0x41			//0: DIS, 1:EN
#define			CODE_Cmd_STRX_OTC_SET_THRESHOLD                         0x42			//Temperature[degC]
#define			CODE_Cmd_SET_STX_ONOFF_DURATION                         0x44			//param1: on����[s]�Cparam2: off����[s]
#define			CODE_Cmd_SET_STX_BURST_DURATION                         0x45			//�A�����M���Ԃ�ݒ�[s]
#define			CODE_Cmd_EPS_RESET                                      0x50			//0: MAIN, 1:SUB
#define			CODE_Cmd_EPS_UVC_ENABLE                                 0x51			//0: DIS, 1:EN
#define			CODE_Cmd_EPS_UVC_SET_BAT_V                              0x52			//
#define			CODE_Cmd_EPS_SELECT                                     0x53			//0:ChA, 1:ChB
#define			CODE_Cmd_ADCS_POWER                                     0x60			//
#define			CODE_Cmd_RW_ENABLE                                      0x61			//
#define			CODE_Cmd_HEATER_POWER                                   0x62			//
#define			CODE_Cmd_RW_POWER                                       0x63			//
#define			CODE_Cmd_MSN_unreg_ENABLE                               0x64			//
#define			CODE_Cmd_MSN_5V_POWER                                   0x65			//
#define			CODE_Cmd_SUBCAM_POWER                                   0x66			//
#define			CODE_Cmd_SF_POWER                                       0x67			//
#define			CODE_Cmd_MTQ_POWER_OBC                                  0x68			//
#define			CODE_Cmd_POWER_SAFE_OFF                                 0x69			//
#define			CODE_Cmd_UNREG_POWER                                    0x70			//
#define			CODE_Cmd_POWER_REBOOT                                   0x71			//
#define			CODE_Cmd_OBC_MTQ_X                                      0x72			//
#define			CODE_Cmd_OBC_MTQ_Y                                      0x73			//
#define			CODE_Cmd_OBC_MTQ_Z                                      0x74			//
#define			CODE_Cmd_STRX_POWER                                     0x75			//
#define			CODE_Cmd_OBC_CAM_POWER                                  0x76			//
#define			CODE_Cmd_OBC_LED_POWER                                  0x77			//
#define			CODE_Cmd_OBC_DR_RECORD                                  0x80			//
#define			CODE_Cmd_OBC_DR_REPLAY_PARAM_SET                        0x81			//
#define			CODE_Cmd_OBC_DR_REPLAY                                  0x82			//
#define			CODE_Cmd_OBC_DR_ERASE                                   0x83			//
#define			CODE_Cmd_MSN_DR_RECORD_ENABLE                           0x84			//
#define			CODE_Cmd_MSN_DR_REPLAY_PARAM_SET                        0x85			//
#define			CODE_Cmd_MSN_DR_REPLAY                                  0x86			//
#define			CODE_Cmd_MSN_DR_ERASE                                   0x87			//
#define			CODE_Cmd_OBC_DR_INITIALIZE                              0x88			//
#define			CODE_Cmd_MSN_DR_INITIALIZE                              0x89			//
#define			CODE_Cmd_OBC_DR_DIRECT_REPLAY                           0x8A			//
#define			CODE_Cmd_MSN_DR_DIRECT_REPLAY                           0x8B			//
