/**
  This file is part of the MSL Refbox.
  
  The MSL Refbox is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License.
  
  The MSL Refbox is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with the MSL Refbox.  If not, see <http://www.gnu.org/licenses/>.
*/

package org.robocup.msl.refbox.applications.refboxTestClient2009;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.AdjustmentEvent;
import java.awt.event.AdjustmentListener;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollBar;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.ScrollPaneConstants;

import org.robocup.msl.refbox.protocol.Protocol2009;

public class RefboxClientApplication extends JFrame {

    // generated id
    private static final long serialVersionUID = -4716940471559249282L;

    private JPanel jPanel = null;

    private JTextArea textArea = null;

    /**
     * This method initializes jPanel	
     * 	
     * @return javax.swing.JPanel	
     */
    private JPanel getJPanel() {
        if (this.jPanel == null) {
            final GridBagConstraints gridBagConstraints8 = new GridBagConstraints();
            gridBagConstraints8.fill = GridBagConstraints.BOTH;
            gridBagConstraints8.gridy = 18;
            gridBagConstraints8.weightx = 1.0;
            gridBagConstraints8.weighty = 1.0;
            gridBagConstraints8.gridwidth = 11;
            gridBagConstraints8.gridx = 4;
            final GridBagConstraints gridBagConstraints7 = new GridBagConstraints();
            gridBagConstraints7.gridx = 4;
            gridBagConstraints7.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints7.ipadx = 10;
            gridBagConstraints7.ipady = 10;
            gridBagConstraints7.fill = GridBagConstraints.BOTH;
            gridBagConstraints7.gridy = 17;
            final GridBagConstraints gridBagConstraints6 = new GridBagConstraints();
            gridBagConstraints6.gridx = 14;
            gridBagConstraints6.fill = GridBagConstraints.BOTH;
            gridBagConstraints6.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints6.ipadx = 10;
            gridBagConstraints6.ipady = 10;
            gridBagConstraints6.gridy = 17;
            final GridBagConstraints gridBagConstraints5 = new GridBagConstraints();
            gridBagConstraints5.gridx = 4;
            gridBagConstraints5.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints5.ipadx = 10;
            gridBagConstraints5.ipady = 10;
            gridBagConstraints5.fill = GridBagConstraints.BOTH;
            gridBagConstraints5.gridy = 16;
            final GridBagConstraints gridBagConstraints4 = new GridBagConstraints();
            gridBagConstraints4.gridx = 14;
            gridBagConstraints4.fill = GridBagConstraints.BOTH;
            gridBagConstraints4.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints4.ipadx = 10;
            gridBagConstraints4.ipady = 10;
            gridBagConstraints4.gridy = 16;
            final GridBagConstraints gridBagConstraints3 = new GridBagConstraints();
            gridBagConstraints3.gridx = 14;
            gridBagConstraints3.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints3.ipadx = 10;
            gridBagConstraints3.ipady = 10;
            gridBagConstraints3.fill = GridBagConstraints.BOTH;
            gridBagConstraints3.gridy = 5;
            final GridBagConstraints gridBagConstraints11 = new GridBagConstraints();
            gridBagConstraints11.gridx = 4;
            gridBagConstraints11.ipadx = 10;
            gridBagConstraints11.ipady = 10;
            gridBagConstraints11.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints11.fill = GridBagConstraints.BOTH;
            gridBagConstraints11.gridy = 5;
            final GridBagConstraints gridBagConstraints1 = new GridBagConstraints();
            gridBagConstraints1.gridx = 5;
            gridBagConstraints1.fill = GridBagConstraints.VERTICAL;
            gridBagConstraints1.ipadx = 10;
            gridBagConstraints1.ipady = 10;
            gridBagConstraints1.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints1.gridwidth = 1;
            gridBagConstraints1.gridy = 5;
            final GridBagConstraints gridBagConstraints = new GridBagConstraints();
            gridBagConstraints.gridx = 5;
            gridBagConstraints.gridwidth = 5;
            gridBagConstraints.gridheight = 6;
            gridBagConstraints.insets = new Insets(10, 10, 10, 10);
            gridBagConstraints.fill = GridBagConstraints.NONE;
            gridBagConstraints.ipadx = 10;
            gridBagConstraints.ipady = 10;
            gridBagConstraints.gridy = 11;
            this.jPanel = new JPanel();
            this.jPanel.setLayout(new GridBagLayout());
            final JScrollPane textScrollPane = new JScrollPane();
            textScrollPane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_ALWAYS);
            textScrollPane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
            textScrollPane.getVerticalScrollBar().addAdjustmentListener(new AdjustmentListener() {
                private int maxValueEver = -1;

                public void adjustmentValueChanged(final AdjustmentEvent e) {
                    if (!e.getValueIsAdjusting()) {
                        final JScrollBar scrollBar = (JScrollBar) e.getAdjustable();
                        if ((scrollBar.getValue() >= this.maxValueEver) || (this.maxValueEver > scrollBar.getMaximum())) {
                            scrollBar.setValue(scrollBar.getMaximum());
                            this.maxValueEver = scrollBar.getValue();
                        }
                    }
                }
            });

            this.jPanel.add(textScrollPane, gridBagConstraints8);
            textScrollPane.getViewport().add(getTextArea(), gridBagConstraints8);
            // jPanel.add(getTextArea(), gridBagConstraints8);
        }
        return this.jPanel;
    }

    /**
     * This method initializes textArea	
     * 	
     * @return javax.swing.JTextArea	
     */
    private JTextArea getTextArea() {
        if (this.textArea == null) {
            this.textArea = new JTextArea();
        }
        return this.textArea;
    }

    /**
     * This is the default constructor
     * @param rbm.getButtonHandler() 
     */
    public RefboxClientApplication() {
        super();
        initialize();
    }

    /**
     * This method initializes this
     * 
     * @return void
     */
    private void initialize() {
        this.setSize(461, 343);
        setContentPane(getJPanel());
        setTitle("Refbox Testclient for " + Protocol2009.class.getSimpleName());
    }

    public void newData(final String s) {
        this.textArea.append(s);

    }

} //  @jve:decl-index=0:visual-constraint="121,31"
