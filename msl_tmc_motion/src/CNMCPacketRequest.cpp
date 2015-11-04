/*
 * CNMCPacketRequest.cpp
 *
 *  Created on: Sep 30, 2015
 *      Author: cnpaul
 */

#include <CNMCPacketRequest.h>

namespace msl_driver
{

	const int CNMCPacket::CMD_GROUP_POS = 1;
	const int CNMCPacket::CMD_POS = 2;
	const int CNMCPacket::DATA_POS = 3;

	CNMCPacket::CNMCPacket()
	{
		this->data = std::make_shared<std::vector<byte> >();
		this->values = std::make_shared<std::vector<VAROBJECT> >();
	}

	CNMCPacket::~CNMCPacket()
	{
	}
	std::shared_ptr<std::vector<byte>> CNMCPacket::getBytes()
	{
		std::shared_ptr<std::vector<byte>> dataList = std::make_shared<std::vector<byte>>();

		dataList->push_back(START_HEADER);
		needQuotes(this->cmdgrp, dataList);
		needQuotes(this->cmd, dataList);

		for(int i = 0; i < this->data->size(); i++)
		{
			byte b = this->data->at(i);
			dataList->push_back(b);
		}
		dataList->push_back(END_HEADER);
		return dataList;

	}
	CNMCPacket::VAROBJECT CNMCPacket::giveVarobject(unsigned char uc ,VAROBJECT::o_t type)
	{
		VAROBJECT o = VAROBJECT();
		o.value.ucValue[0] =  uc;
		o.objectType = VAROBJECT::UNSIGNEDCHAR;
		return o;
	}
	CNMCPacket::VAROBJECT CNMCPacket::giveVarobject(int uc ,VAROBJECT::o_t type)
	{
		VAROBJECT o = VAROBJECT();
		o.value.dblValue =  uc;
		o.objectType = VAROBJECT::INT;
		return o;
	}
	CNMCPacket::VAROBJECT CNMCPacket::giveVarobject(short uc ,VAROBJECT::o_t type)
	{
		VAROBJECT o = VAROBJECT();
		o.value.shValue =  uc;
		o.objectType = VAROBJECT::SHORT;
		return o;
	}
	CNMCPacket::VAROBJECT CNMCPacket::giveVarobject(signed char uc ,VAROBJECT::o_t type)
	{
		VAROBJECT o = VAROBJECT();
		o.value.sByte =  uc;
		o.objectType = VAROBJECT::SIGNEDCHAR;
		return o;
	}
	void CNMCPacket::needQuotes(byte b, std::shared_ptr<std::vector<byte>> list)
	{

		if(b == START_HEADER || b == END_HEADER || b == QUOTE)
		{
			list->push_back(QUOTE);
			list->push_back(b);
		}
		else
		{
			list->push_back(b);
		}
	}

	bool CNMCPacket::isExpectedResponse(CNMCPacket response)
	{
		bool retVal = (	this->cmd == response.cmd
						&& this->cmdgrp+1 == response.cmdgrp
						&& this->data->size() == response.data->size()
				);
		for (int i=0; i<this->data->size() && retVal; i++)
		{
			retVal &= this->data->at(i) == response.data->at(i);
		}
		return retVal;
	}

	static CNMCPacket getInstance(byte raw[], int size)
	{
		if(size < 3)
		{
			std::cerr << "Invalid CNMC packet" << std::endl;
		}
		switch ((CNMCPacket::CommandGroup)raw[CNMCPacket::CMD_GROUP_POS])
		{
			case CNMCPacket::CommandGroup::Configure:
				return CNMCPacketConfigure(raw);

			case CNMCPacket::CommandGroup::Control:
				return CNMCPacketControl(raw);

			case CNMCPacket::CommandGroup::Request:
				return CNMCPacketRequest(raw);

			case CNMCPacket::CommandGroup::CtrlConfigure:
				return CNMCPacketCtrlConfigure(raw);

			case CNMCPacket::CommandGroup::CtrlConfigureResponse:
				return CNMCPacketCtrlConfigureResponse(raw);

			case CNMCPacket::CommandGroup::ErrorResponse:
				return CNMCPacketError(raw);

			case CNMCPacket::CommandGroup::RequestResponse:
				return CNMCPacketRequestResponse(raw, size);

			case CNMCPacket::CommandGroup::ConfigureResponse:
				return CNMCPacketConfigureResponse(raw, size);

			default:
				std::cerr << "UNKNOWN CNMC packet" << std::endl;
		}
	}
	//TODO --> useless wahrscheinlich?
	void CNMCPacket::add(byte b)
	{
		if((b == QUOTE) || (b == START_HEADER) || (b == END_HEADER))
		{
			this->data->push_back(QUOTE);
		}
		this->data->push_back(b);
	}
	//TODO --> bytes[1] --> check ob es überhaupt gesetzt wurde!
	void CNMCPacket::add(VAROBJECT bytes)
	{
		switch (bytes.objectType)
		{
			case VAROBJECT::INT:
			{
				if((bytes.value.dblValue == QUOTE) || (bytes.value.dblValue == START_HEADER) || (bytes.value.dblValue  == END_HEADER))
				{
					this->data->push_back(QUOTE);
				}
				this->data->push_back((byte)bytes.value.dblValue);
			}
				break;
			case VAROBJECT::SHORT:
			{
				if((bytes.value.shValue == QUOTE) || (bytes.value.shValue == START_HEADER) || (bytes.value.shValue  == END_HEADER))
				{
					this->data->push_back(QUOTE);
				}
				this->data->push_back((byte)bytes.value.shValue);
			}
				break;
			case VAROBJECT::SIGNEDCHAR:
			{
				if((bytes.value.sByte == QUOTE) || (bytes.value.sByte == START_HEADER) || (bytes.value.sByte  == END_HEADER))
				{
					this->data->push_back(QUOTE);
				}
				this->data->push_back((byte)bytes.value.sByte);
			}
				break;
			case VAROBJECT::UNSIGNEDCHAR:
			{
				int a = 0;
				for(unsigned char uc : bytes.value.ucValue)
				{
					if((uc == QUOTE) || (uc == START_HEADER) || (uc == END_HEADER))
					{
						this->data->push_back(QUOTE);
					}
					this->data->push_back(bytes.value.ucValue[a]);
					a++;
				}
			}
				break;
			default:
				std::cerr << "SOMETHING GOES WRONG " << std::endl;
		}
	}

	//**** CONVERT METHODS ****//
	CNMCPacket::VAROBJECT CNMCPacket::convertByteToShort(byte data[], int start)
	{
		short sh = (short)(((unsigned short)(data[start]) << 8 ) + (unsigned char)data[start+1]);

		VAROBJECT o = VAROBJECT();
		o.value.shValue = sh;
		o.objectType = VAROBJECT::SHORT;

		return o;

	}
	CNMCPacket::VAROBJECT CNMCPacket::convertShortToByte(short data)
	{
		VAROBJECT o = VAROBJECT();
		o.value.ucValue[0] =  data & 0xff;
		o.value.ucValue[1] =  (data >> 8) & 0xff;
		o.objectType = VAROBJECT::UNSIGNEDCHAR;
		return o;
	}
	CNMCPacket::VAROBJECT CNMCPacket::convertByteToInt(byte data[], int start)
	{
		int i = 0;
		i = (i << 8) + data[start+3];
		i = (i << 8) + data[start+2];
		i = (i << 8) + data[start+1];
		i = (i << 8) + data[start];


		VAROBJECT o = VAROBJECT();
		o.value.dblValue = i;
		o.objectType = VAROBJECT::INT;
		return o;
	}

	CNMCPacket::VAROBJECT CNMCPacket::convertIntToByte(int data)
	{
		byte* arrayOfByte =  new byte(4);


		VAROBJECT o = VAROBJECT();
		o.objectType = VAROBJECT::UNSIGNEDCHAR;

		for (int i = 0; i < 4; i++)
		{
			o.value.ucValue[3 - i] = (data >> (i * 8));
		}
		return o;
	}

	//**** CONVERT METHODS END ****//


	///############## CNMCPacketConfigure ##############///
	CNMCPacketConfigure::CNMCPacketConfigure()
	{
		this->cmdgrp = (byte)CommandGroup::Configure;
	}
	CNMCPacketConfigure::~CNMCPacketConfigure()
	{
	}
	CNMCPacketConfigure::CNMCPacketConfigure(byte raw[])
	{
		if((CNMCPacket::CommandGroup)raw[CNMCPacket::CMD_GROUP_POS] != CommandGroup::Configure)
		{
			std::cerr << "No configuration packet!" << std::endl;
		}
		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();

		if(((this->cmd & 0xfe) == 0x50) || (this->cmd == (byte)ConfigureCmd::IOPort))
		{
			VAROBJECT o = VAROBJECT();
			o.value.shValue = (short) 0 ;
			o.objectType = VAROBJECT::SHORT;
			this->values->push_back(o);
		}
		else
		{
			VAROBJECT o = VAROBJECT();
			o.value.ucValue[0] = (unsigned char)raw[DATA_POS] ;
			o.objectType = VAROBJECT::UNSIGNEDCHAR;

			this->values->push_back(o);
		}

		this->values->push_back(convertByteToShort(raw, DATA_POS+1));
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, byte val)
	{
		this->cmd = (byte) cmd;
		this->data->clear();
		this->values->clear();

		VAROBJECT o = VAROBJECT();
		o.value.ucValue[0] = (unsigned char) val;
		o.objectType = VAROBJECT::UNSIGNEDCHAR;

		add(o);
		this->values->push_back(o);

	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, short val)
	{
		this->cmd = (byte) cmd;
		this->data->clear();
		this->values->clear();
		add(convertShortToByte(val));

		VAROBJECT o = VAROBJECT();
		o.value.shValue = (short) val;
		o.objectType = VAROBJECT::SHORT;
		this->values->push_back(o);
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, std::shared_ptr<std::vector<byte>> vals)
	{
		this->cmd = (byte) cmd;
		this->data->clear();
		this->values->clear();
		for(int i = 0; i < vals->size(); i++)
		{
			byte b = vals->at(i);
			VAROBJECT o = VAROBJECT();
			o.value.ucValue[0] = (unsigned char) b;
			o.objectType = VAROBJECT::UNSIGNEDCHAR;
			add(o);
			this->values->push_back(o);

		}
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, std::shared_ptr<std::vector<sbyte>> vals)
	{
		this->cmd = (byte) cmd;
		this->data->clear();
		this->values->clear();
		for(int i = 0; i < vals->size(); i++)
		{
			byte b = vals->at(i);
			VAROBJECT o = VAROBJECT();
			o.value.ucValue[0] = (unsigned char) b;
			o.objectType = VAROBJECT::UNSIGNEDCHAR;
			add(o);
			this->values->push_back(o);

		}
	}
	///############## CNMCPacketControl ##############///
	CNMCPacketControl::CNMCPacketControl()
	{
		this->cmdgrp = (byte)CommandGroup::Control;
	}
	CNMCPacketControl::~CNMCPacketControl()
	{
	}
	CNMCPacketControl::CNMCPacketControl(byte raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::Control) {
			std::cerr << "No control packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		if (this->cmd != (byte)ControlCmd::SetAllRPM) {
			std::cerr << "Unsuppported reply (Control): " << std::endl;
		}

		this->values->clear();

		VAROBJECT o1 = convertByteToShort(raw, DATA_POS);
		this->values->push_back(o1);

		VAROBJECT o2 = convertByteToShort(raw, DATA_POS+2);
		this->values->push_back(o2);

		VAROBJECT o3 = convertByteToShort(raw, DATA_POS+4);
		this->values->push_back(o3);

		VAROBJECT o4 = VAROBJECT();
		o4.value.ucValue[0] = raw[DATA_POS+6];
		o4.objectType = VAROBJECT::UNSIGNEDCHAR;
		this->values->push_back(o4);
	}
	void CNMCPacketControl::setData(ControlCmd cmd, short rpm1, short rpm2, short rpm3)
	{
		this->cmd = (byte)cmd;

		this->data->clear();

		add(convertShortToByte(rpm1));
		add(convertShortToByte(rpm2));
		add(convertShortToByte(rpm3));

		VAROBJECT o1 = VAROBJECT();
		o1.value.shValue = (short) rpm1;
		o1.objectType = VAROBJECT::SHORT;
		this->values->push_back(o1);

		VAROBJECT o2 = VAROBJECT();
		o2.value.shValue  = (short) rpm2;
		o2.objectType = VAROBJECT::SHORT;
		this->values->push_back(o2);

		VAROBJECT o3 = VAROBJECT();
		o3.value.shValue  = (short) rpm3;
		o3.objectType = VAROBJECT::SHORT;
		this->values->push_back(o3);
	}


	///############## CNMCPacketRequest ##############///
	CNMCPacketRequest::CNMCPacketRequest()
	{
		this->cmdgrp = (byte)CommandGroup::Request;
	}
	CNMCPacketRequest::~CNMCPacketRequest()
	{
	}
	CNMCPacketRequest::CNMCPacketRequest(byte raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::Request)
		{
			std::cerr << "No request packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();

		if (this->cmd < 0x40)
		{
			VAROBJECT o4 = VAROBJECT();
			o4.value.ucValue[0] = raw[DATA_POS];
			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
			this->values->push_back(o4);

			VAROBJECT o2 = convertByteToShort(raw, DATA_POS+1);;
			this->values->push_back(o2);

			if (raw[DATA_POS] == 0)
			{
				VAROBJECT o3 = convertByteToShort(raw, DATA_POS+3);
				this->values->push_back(o3);

				VAROBJECT o5 = convertByteToShort(raw, DATA_POS+5);
				this->values->push_back(o5);
			}

		}
		else if ((this->cmd & 0xfe) == 0x40)
		{
			VAROBJECT o4 = VAROBJECT();
			o4.value.ucValue[0] = raw[DATA_POS];
			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
			this->values->push_back(o4);

			VAROBJECT o = VAROBJECT();
			o.value.ucValue[0]  = raw[DATA_POS+1];
			o.objectType = VAROBJECT::UNSIGNEDCHAR;
			this->values->push_back(o);

		}
		else if (this->cmd == 0x42)
		{
			VAROBJECT o4 = VAROBJECT();
			o4.value.ucValue[0]  = raw[DATA_POS];
			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
			this->values->push_back(o4);

			VAROBJECT o3 = convertByteToShort(raw, DATA_POS+1);
			this->values->push_back(o3);

			VAROBJECT o5 = convertByteToShort(raw, DATA_POS+3);
			this->values->push_back(o5);


		}
		else
		{
			std::cerr << "Unsuppported reply (Request): " << std::endl;
		}
	}
	void CNMCPacketRequest::setData(RequestCmd cmd)
	{
		this->cmd = (byte)cmd;
		this->data->clear();
	}
	void CNMCPacketRequest::setData(RequestCmd cmd, short val)
	{
		this->cmd = (byte)cmd;

		this->data->clear();
		add(convertShortToByte(val));

		VAROBJECT o3 = VAROBJECT();
		o3.value.shValue = val;
		o3.objectType = VAROBJECT::SHORT;
		this->values->push_back(o3);


	}

	///############## CNMCPacketCtrlConfigureResponse ##############///
	CNMCPacketCtrlConfigureResponse::CNMCPacketCtrlConfigureResponse()
	{
		this->cmdgrp = (byte)CommandGroup::CtrlConfigureResponse;
	}
	CNMCPacketCtrlConfigureResponse::~CNMCPacketCtrlConfigureResponse()
	{
		VAROBJECT giveVarobject(unsigned char uc ,VAROBJECT::o_t type);
			VAROBJECT giveVarobject(int uc ,VAROBJECT::o_t type);
			VAROBJECT giveVarobject(short uc ,VAROBJECT::o_t type);
			VAROBJECT giveVarobject(signed char uc ,VAROBJECT::o_t type);
	}
	CNMCPacketCtrlConfigureResponse::CNMCPacketCtrlConfigureResponse(byte raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::CtrlConfigureResponse)
		{
			std::cerr << "No control configure response packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();
		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::Commit) return;
		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxAcceleration ||
			(CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxDecceleration ||
			(CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxRotForce)
		{
			VAROBJECT o3 = convertByteToInt(raw,DATA_POS);
			this->values->push_back(o3);

			for(int i=0; i<4; i++)
			{
				add(giveVarobject(raw[DATA_POS+i], VAROBJECT::UNSIGNEDCHAR));
			}
			return;
		}
		VAROBJECT o3 = convertByteToShort(raw,DATA_POS);
		this->values->push_back(o3);


		add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
		add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));

		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::FailSafeValues)
		{

			VAROBJECT o3 = convertByteToShort(raw,DATA_POS+2);
			this->values->push_back(o3);

			add(giveVarobject(raw[DATA_POS+2], VAROBJECT::UNSIGNEDCHAR));
			add(giveVarobject(raw[DATA_POS+3], VAROBJECT::UNSIGNEDCHAR));

			VAROBJECT o4 =  convertByteToShort(raw,DATA_POS+4);
			this->values->push_back(o4);

			add(giveVarobject(raw[DATA_POS+4], VAROBJECT::UNSIGNEDCHAR));
			add(giveVarobject(raw[DATA_POS+5], VAROBJECT::UNSIGNEDCHAR));
		}

	}

	///############## CNMCPacketCtrlConfigure ##############///
	CNMCPacketCtrlConfigure::CNMCPacketCtrlConfigure()
	{
		this->cmdgrp = (byte)CommandGroup::CtrlConfigure;
	}
	CNMCPacketCtrlConfigure::~CNMCPacketCtrlConfigure()
	{

	}
	CNMCPacketCtrlConfigure::CNMCPacketCtrlConfigure(byte raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::CtrlConfigure)
		{
			std::cerr << "No control configure packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();

		VAROBJECT o3 = giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR);
		this->values->push_back(o3);

		VAROBJECT o = convertByteToShort(raw,DATA_POS);
		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd)
	{
		this->cmd = (byte)cmd;
		this->data->clear();
	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd, short val)
	{
		this->cmd = (byte)cmd;
		this->data->clear();
		add(convertShortToByte(val));

		VAROBJECT o = VAROBJECT();
		o.value.shValue = val;
		o.objectType = VAROBJECT::SHORT;
		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmdl, int val)
	{
		this->cmd = (byte)cmd;
		this->data->clear();
		add(convertIntToByte(val));


		VAROBJECT o = VAROBJECT();
		o.value.dblValue = val;
		o.objectType = VAROBJECT::INT;
		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd, std::shared_ptr<std::vector<short>> vals)
	{

		this->cmd = (byte)cmd;

		this->data->clear();
		for(int i = 0; i < vals->size(); i++)
		{
			add(convertShortToByte(vals->at(i)));
			VAROBJECT o = giveVarobject(vals->at(i), VAROBJECT::SHORT);
			this->values->push_back(o);
		}

	}

	///############## CNMCPacketError ##############///

	CNMCPacketError::CNMCPacketError()
	{
		this->cmdgrp = (byte)CommandGroup::ErrorResponse;
	}
	CNMCPacketError::~CNMCPacketError()
	{
	}
	CNMCPacketError::CNMCPacketError(byte raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::ErrorResponse)
		{
			std::cerr << "No error packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();
		std::cerr << "No error packet!" << this->toString() << std::endl;
	}
	std::string CNMCPacketError::toString()
	{
		std::string reason = "";
		switch ((ErrorCmd)this->cmd) {
			case ErrorCmd::CycleOverTime:
				reason = "cycle overtime";
				break;
			case ErrorCmd::UnknownError:
				reason = "unknown";
				break;
			case ErrorCmd::UnknownCmd:
				reason = "unknown Cmd";
				break;
			case ErrorCmd::ParamError:
				reason = "Parameter Error";
				break;
			case ErrorCmd::OutOfRange:
				reason = "parameter out of range";
				break;
			default:
				break;
		}
		return std::string("ErrorReponse: ") + std::to_string(static_cast<unsigned>(this->cmd)) + std::string("(") + reason + std::string(")");
	}

	///############## CNMCPacketRequestResponse ##############///
	CNMCPacketRequestResponse::CNMCPacketRequestResponse()
	{
		this->cmdgrp = (byte) CommandGroup::RequestResponse;
	}
	CNMCPacketRequestResponse::~CNMCPacketRequestResponse()
	{

	}
	CNMCPacketRequestResponse::CNMCPacketRequestResponse(byte raw[], int size)
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::RequestResponse)
		{
			std::cerr << "No RequestResponse packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();
		switch(this->cmd)
		{
			case RequestCmd::PathVector:
			{
				add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o = convertByteToShort(raw,DATA_POS);
				this->values->push_back(o);

				add(giveVarobject(raw[DATA_POS+2], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+3], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o1 = convertByteToShort(raw,DATA_POS+2);
				this->values->push_back(o1);

				add(giveVarobject(raw[DATA_POS+4], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+5], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o2 = convertByteToShort(raw,DATA_POS+4);
				this->values->push_back(o2);
			}
				break;
			case RequestCmd::PathAndPWM:
			{
				add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o3 = convertByteToShort(raw,DATA_POS);
				this->values->push_back(o3);

				add(giveVarobject(raw[DATA_POS+2], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+3], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o4 = convertByteToShort(raw,DATA_POS+2);
				this->values->push_back(o4);

				add(giveVarobject(raw[DATA_POS+4], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+5], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o5 = convertByteToShort(raw,DATA_POS+4);
				this->values->push_back(o5);

				add(giveVarobject(raw[DATA_POS+6], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+7], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o6 = convertByteToShort(raw,DATA_POS+6);
				this->values->push_back(o6);

				add(giveVarobject(raw[DATA_POS+8], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+9], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o7 = convertByteToShort(raw,DATA_POS+8);
				this->values->push_back(o7);

				add(giveVarobject(raw[DATA_POS+10], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+11], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o8 = convertByteToShort(raw,DATA_POS+10);
				this->values->push_back(o8);
			}
				break;

			case RequestCmd::EncoderTicksRel:
			{
				add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+2], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+3], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o9 = convertByteToInt(raw,DATA_POS);
				this->values->push_back(o9);

				add(giveVarobject(raw[DATA_POS+4], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+5], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+6], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+7], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o10 = convertByteToInt(raw,DATA_POS+4);
				this->values->push_back(o10);

				add(giveVarobject(raw[DATA_POS+8], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+9], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+10], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+11], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o11 = convertByteToInt(raw,DATA_POS+8);
				this->values->push_back(o11);
			}
				break;
			case RequestCmd::MotorCurrent:
			{
				add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));


				VAROBJECT o12 = convertByteToShort(raw,DATA_POS);
				this->values->push_back(o12);

				add(giveVarobject(raw[DATA_POS+2], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+3], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o13 = convertByteToShort(raw,DATA_POS+2);
				this->values->push_back(o13);

				add(giveVarobject(raw[DATA_POS+4], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+5], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o14 = convertByteToShort(raw,DATA_POS+4);
				this->values->push_back(o14);
			}
				break;
			case RequestCmd::BatteryVoltage:
			{
				add(giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR));
				add(giveVarobject(raw[DATA_POS+1], VAROBJECT::UNSIGNEDCHAR));

				VAROBJECT o15 = convertByteToShort(raw,DATA_POS);
				this->values->push_back(o15);
			}
				break;
			case RequestCmd::ReadOdoLog:
			{
				for(int i= DATA_POS; i< size; i+=2)
				{
					add(giveVarobject(raw[i], VAROBJECT::UNSIGNEDCHAR));
					add(giveVarobject(raw[i+1], VAROBJECT::UNSIGNEDCHAR));
					VAROBJECT o = convertByteToShort(raw,i);
					this->values->push_back(o);
				}
			}
				break;
			default:
				std::cerr << "Never saw this response package" << std::endl;
		}
	}

	///############## CNMCPacketConfigureResponse ##############///
	CNMCPacketConfigureResponse::CNMCPacketConfigureResponse()
	{
		this->cmdgrp = (byte) CommandGroup::ConfigureResponse;
	}
	CNMCPacketConfigureResponse::~CNMCPacketConfigureResponse()
	{
	}
	CNMCPacketConfigureResponse::CNMCPacketConfigureResponse(byte raw[], int size)
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::ConfigureResponse)
		{
			std::cerr << "No ConfigureResponse packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		this->values->clear();

		for(int i = DATA_POS; i < size; i++)
		{
			add(giveVarobject(raw[i], VAROBJECT::UNSIGNEDCHAR));
		}
	}
} /* namespace msl_driver */
