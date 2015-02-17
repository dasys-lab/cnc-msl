using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace CNMC_Debug_Client
{
    public partial class GUI : Form
    {

        private CNMC cnmc;

        public GUI(CNMC controller)
        {
            this.cnmc = controller;
            this.cnmc.setGui(this);
            InitializeComponent();

            pwm_m1_soll.Text = "0";
            pwm_m2_soll.Text = "0";
            pwm_m3_soll.Text = "0";

            pwm_m1_ist.Text = "unk";
            pwm_m2_ist.Text = "unk";
            pwm_m3_ist.Text = "unk";

            messagesWindow.Text = "";

        }

        public bool isSendPeriodicPWM()
        {
            if (pwm_keep_sending != null)
            {
                return pwm_keep_sending.Checked;
            }

            return false;
        }

        public bool isUpdateTicks()
        {
            if (update_ticks != null)
            {
                return update_ticks.Checked;
            }

            return false;
        }

        public bool isUpdateCurrent()
        {
            if (current_update != null)
            {
                return current_update.Checked;
            }

            return false;
        }

        public bool isUpdateVoltage()
        {
            if (voltage_update != null)
            {
                return voltage_update.Checked;
            }

            return false;
        }

        public void addSendMessage(String line)
        {
            if (show_send != null && show_send.Checked)
            {
                addMessage("<< " + line);
            }
        }

        public void addReceiveMessage(String line)
        {
            if (show_recv != null && show_recv.Checked)
            {
                addMessage(">> " + line);
            }
        }

        public void addNoticeMessage(String line)
        {
            if (show_note != null && show_note.Checked)
            {
                addMessage("!! " + line);
            }
        }

        private delegate void messageDelegate(String line);
        private void addMessage(String line)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new messageDelegate(addMessage), line);
            }
            else
            {
                if (messagesWindow != null)
                {
                    messagesWindow.AppendText(line + "\n");
                    messagesWindow.ScrollToCaret();
                }
            }
        }

        private void pwm_update_Click(object sender, EventArgs e)
        {
            cnmc.requestAllPWM();
        }

        private delegate void pwmDelegate(Int16 pwm1, Int16 pwm2, Int16 pwm3);
        public void updatePWM(Int16 pwm1, Int16 pwm2, Int16 pwm3)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new pwmDelegate(updatePWM), new object[] { pwm1, pwm2, pwm3 });
            }
            else
            {
                pwm_m1_ist.Text = Convert.ToString(pwm1);
                pwm_m2_ist.Text = Convert.ToString(pwm2);
                pwm_m3_ist.Text = Convert.ToString(pwm3);
            }
        }

        private delegate void currentDelegate(UInt32 current1, UInt32 current2, UInt32 current3);
        public void updateCurrent(UInt32 current1, UInt32 current2, UInt32 current3)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new currentDelegate(updateCurrent), new object[] { current1, current2, current3 });
            }
            else
            {
                if (current_motor1 != null && current_motor2 != null && current_motor3 != null)
                {
                    current_motor1.Text = Convert.ToString(current1 / 100f);
                    current_motor2.Text = Convert.ToString(current2 / 100f);
                    current_motor3.Text = Convert.ToString(current3 / 100f);
                }
            }
        }

        private delegate void voltageDelegate(UInt16 voltage);
        public void updateSupplyVoltage(UInt16 voltage)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new voltageDelegate(updateSupplyVoltage), voltage);
            }
            else
            {
                if (voltage_supply != null)
                {
                    voltage_supply.Text = Convert.ToString(voltage / 100f);
                }
            }
        }

        private delegate void updateEncoderDelegate(Int32 ticks1, Int32 ticks2, Int32 ticks3);
        public void updateEncoderRelative(Int32 ticks1, Int32 ticks2, Int32 ticks3)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new updateEncoderDelegate(updateEncoderRelative), new object[] {ticks1, ticks2, ticks3});
            }
            else
            {
                if (ticks_m1 != null && ticks_m2 != null && ticks_m3 != null)
                {
                    ticks_m1.Text = Convert.ToString(ticks1);
                    ticks_m2.Text = Convert.ToString(ticks2);
                    ticks_m3.Text = Convert.ToString(ticks3);
                }
            }
        }

        public void pwm_send_pwm()
        {
            pwm_commit_Click(null, null);
        }

        private void pwm_commit_Click(object sender, EventArgs e)
        {
            cnmc.setAllPWM(Convert.ToInt16(pwm_m1_soll.Text), Convert.ToInt16(pwm_m2_soll.Text), Convert.ToInt16(pwm_m3_soll.Text)); 
        }

        private void pwm_auto_update_CheckedChanged(object sender, EventArgs e)
        {
            
        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {

        }

        private void update_supply_voltage_Click(object sender, EventArgs e)
        {
            cnmc.requestVoltage();
        }

        private void update_ticks_rel_Click(object sender, EventArgs e)
        {
            cnmc.requestTicksRel();
        }

        private void stop_motors_Click(object sender, EventArgs e)
        {
            pwm_m1_soll.Text = Convert.ToString(0);
            pwm_m2_soll.Text = Convert.ToString(0);
            pwm_m3_soll.Text = Convert.ToString(0);
            cnmc.setAllPWM(0, 0, 0);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            cnmc.setCycleTime(Convert.ToByte(cycle_time.Text));
        }


    }
}
