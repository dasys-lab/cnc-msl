package de.lab.ui;

import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.regex.Pattern;

import javax.swing.GroupLayout;
import javax.swing.GroupLayout.Alignment;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.LayoutStyle.ComponentPlacement;

import de.lab.net.Communication;
import de.lab.net.ReaderWriter;

public class MainWindow {

	private JFrame frmVisualizer;
	private JTextField txtdevttyusb;
	private JButton btnConnect;
	private JTextField rangeText;
	private ReaderWriter reader;
	private JButton sendRange;
	private JLabel lblEntfernung;
	private JLabel lblRange;
	private JTextField gainText;
	private JLabel lblGain;
	private JButton sendGain;
	private JLabel label;
	private Plotter plotter;

	/**
	 * Initialize the contents of the frame.
	 * @wbp.parser.entryPoint
	 */
	public void initialize() {
		frmVisualizer = new JFrame();
		frmVisualizer.setTitle("Visualizer");
		frmVisualizer.setBounds(100, 100, 520, 350);
		frmVisualizer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		plotter = new Plotter();
		
		btnConnect = new JButton("Connect");
		btnConnect.addMouseListener(new MouseAdapter() {
			
			@Override
			public void mouseClicked(MouseEvent e) {
				if(Pattern.matches("/dev/ttyUSB[0-9]", txtdevttyusb.getText()))
				{
					Communication comm = new Communication(txtdevttyusb.getText());
					try {
						comm.connect();
						reader = new ReaderWriter(comm.getPort(), plotter, label, lblEntfernung, 0);
						reader.start();
						txtdevttyusb.setEnabled(false);
						btnConnect.setEnabled(false);
						rangeText.setEnabled(true);
						sendRange.setEnabled(true);
						gainText.setEnabled(true);
						sendGain.setEnabled(true);
					} catch (PortInUseException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					} catch (UnsupportedCommOperationException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
				}				
			}
		});
		
		txtdevttyusb = new JTextField();
		txtdevttyusb.setText("/dev/ttyUSB0");
		txtdevttyusb.setColumns(10);
		
		rangeText = new JTextField();
		rangeText.setEnabled(false);
		
		rangeText.setColumns(10);
		
		sendRange = new JButton("Send Range");
		sendRange.setEnabled(false);
		sendRange.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				if(!("".equals(rangeText.getText())))
				{
					reader.writeData(2);
					reader.writeData(Integer.valueOf(rangeText.getText()));
				}
			}
		});
		
		lblEntfernung = new JLabel("Entfernung: ");
		
		lblRange = new JLabel("Range:");
		
		gainText = new JTextField();
		gainText.setEnabled(false);
		gainText.setColumns(10);
		
		lblGain = new JLabel("Gain:");
		
		sendGain = new JButton("Send Gain");
		sendGain.addMouseListener(new MouseAdapter() {
			@Override
			public void mouseClicked(MouseEvent e) {
				if(!("".equals(gainText.getText())))
				{
					reader.writeData(1);
					reader.writeData(Integer.valueOf(gainText.getText()));
				}
			}
		});
		
		sendGain.setEnabled(false);
		
		label = new JLabel(plotter);
		GroupLayout groupLayout = new GroupLayout(frmVisualizer.getContentPane());
		groupLayout.setHorizontalGroup(
			groupLayout.createParallelGroup(Alignment.LEADING)
				.addGroup(groupLayout.createSequentialGroup()
					.addContainerGap()
					.addGroup(groupLayout.createParallelGroup(Alignment.LEADING)
						.addComponent(lblEntfernung)
						.addGroup(groupLayout.createSequentialGroup()
							.addGroup(groupLayout.createParallelGroup(Alignment.LEADING)
								.addGroup(groupLayout.createSequentialGroup()
									.addComponent(lblRange)
									.addPreferredGap(ComponentPlacement.RELATED)
									.addComponent(rangeText, GroupLayout.PREFERRED_SIZE, 62, GroupLayout.PREFERRED_SIZE)
									.addPreferredGap(ComponentPlacement.RELATED)
									.addComponent(sendRange))
								.addComponent(txtdevttyusb, 247, 247, 247))
							.addPreferredGap(ComponentPlacement.RELATED)
							.addGroup(groupLayout.createParallelGroup(Alignment.LEADING)
								.addGroup(groupLayout.createSequentialGroup()
									.addComponent(lblGain)
									.addPreferredGap(ComponentPlacement.RELATED)
									.addComponent(gainText, GroupLayout.PREFERRED_SIZE, 57, GroupLayout.PREFERRED_SIZE)
									.addGap(18)
									.addComponent(sendGain))
								.addComponent(btnConnect)))
						.addComponent(label, GroupLayout.PREFERRED_SIZE, 423, GroupLayout.PREFERRED_SIZE))
					.addGap(11))
		);
		groupLayout.setVerticalGroup(
			groupLayout.createParallelGroup(Alignment.LEADING)
				.addGroup(groupLayout.createSequentialGroup()
					.addGap(8)
					.addGroup(groupLayout.createParallelGroup(Alignment.BASELINE)
						.addComponent(txtdevttyusb, GroupLayout.PREFERRED_SIZE, GroupLayout.DEFAULT_SIZE, GroupLayout.PREFERRED_SIZE)
						.addComponent(btnConnect))
					.addGap(15)
					.addGroup(groupLayout.createParallelGroup(Alignment.BASELINE)
						.addComponent(lblRange)
						.addComponent(rangeText, GroupLayout.PREFERRED_SIZE, 25, GroupLayout.PREFERRED_SIZE)
						.addComponent(sendRange)
						.addComponent(lblGain)
						.addComponent(gainText, GroupLayout.PREFERRED_SIZE, 25, GroupLayout.PREFERRED_SIZE)
						.addComponent(sendGain))
					.addPreferredGap(ComponentPlacement.RELATED)
					.addComponent(label, GroupLayout.DEFAULT_SIZE, 199, Short.MAX_VALUE)
					.addGap(18)
					.addComponent(lblEntfernung)
					.addContainerGap())
		);
		frmVisualizer.getContentPane().setLayout(groupLayout);
		frmVisualizer.setVisible(true);
	}

	public JButton getBtnConnect() {
		return btnConnect;
	}
	public JTextField getTextField_1() {
		return rangeText;
	}
	public JButton getBtnSend() {
		return sendRange;
	}
	public JButton getButton() {
		return sendGain;
	}
	public JLabel getLabel() {
		return label;
	}
}
