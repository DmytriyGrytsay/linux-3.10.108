/*
 * (C) Copyright 2011
 * Ricado Ribalda - ricardo.ribalda@gmail.com
 * QTechnology  http://qtec.com/
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include "q5_sdregs.h"

#define DRIVER_NAME "q5_sd"

#define MAX_FREQ_DIV CLOCK_DIVIDE_MASK
#define MIN_FREQ_DIV 2
#define MAX_BLK_SIZE (DATA_RECV_LEN_MASK+1)
#define MAX_BLK_COUNT DMA_LENGTH_MASK
#define SD_HOST_CMD 0x40
#define SD_TIMEOUT (HZ>>2)
#define SD_TIMEOUT_MS 50

enum req_mode {IDLE, IN_CMD, IN_DMA, IN_STOP};

struct q5_sd_priv{
	int debug;
	uint32_t freq;
	struct resource mem;
	void *iomem;
	int irq;
	struct mmc_host *mmc_host;
	struct mmc_request *req;
	struct mutex req_mutex;
	struct platform_device *pdev;
	struct work_struct cmd_completed;
	struct work_struct dma_completed;
	struct timer_list timeout_cmd_timer;
	struct timer_list timeout_dma_timer;
	enum req_mode mode;
};

static inline int q5_sd_read(struct q5_sd_priv *priv, uint32_t addr, uint32_t *value){
	*value=ioread32be(priv->iomem+addr);
	if (priv->debug)
		dev_err(&priv->pdev->dev, "R 0x%.4x 0x%.8x\n",addr,*value);
	return 0;
};

static inline int q5_sd_write(struct q5_sd_priv *priv, uint32_t addr, uint32_t value){
	if (priv->debug)
		dev_err(&priv->pdev->dev, "W 0x%.4x 0x%.8x\n",addr,value);
	iowrite32be(value,priv->iomem+addr);
	return 0;
};

static inline int q5_sd_write_cmd(struct q5_sd_priv *priv, uint32_t value){
	uint32_t status;
	unsigned long expiration=jiffies+SD_TIMEOUT;
	do{
		q5_sd_read(priv,QSD_STATUS,&status);
		if (time_before(expiration,jiffies)){
			dev_err(&priv->pdev->dev, "CMD TX full Timeout Status=0x%.2x",status);
			return -ETIMEDOUT;
		}
	}while(GETREG(status,CMD_TX_FULL));

	q5_sd_write(priv,QSD_CMD_DATA,value);
	return 0;
};

static int q5_sd_read_cmd(struct q5_sd_priv *priv, uint8_t *data){
	uint32_t status;
	unsigned long expiration=jiffies+SD_TIMEOUT;

	while (1){
		q5_sd_read(priv,QSD_STATUS,&status);

		if (GETREG(status,CMD_RX_EMPTY)==0)
			break;

		if (time_before(expiration,jiffies)){
			dev_err(&priv->pdev->dev, "CMD Timeout Status=0x%.2x",status);
			return -ETIMEDOUT;
		}
	}

	q5_sd_read(priv,QSD_CMD_DATA,&status);
	*data=status&0xff;
	if (priv->debug)
		dev_err(&priv->pdev->dev, "CMD R=0x%.2x (%c)\n",status,status);
	return 0;
};

static int q5_sd_get_ro(struct mmc_host *mmc)
{
	struct q5_sd_priv *priv=mmc_priv(mmc);
	uint32_t reg;

	if (priv->debug)
		dev_err(&priv->pdev->dev,
			"get_ro\n");

	q5_sd_read(priv,QSD_STATUS,&reg);
	return GETREG(reg,CARD_PROTECTED);
}

static int q5_sd_get_cd(struct mmc_host *mmc)
{
	struct q5_sd_priv *priv=mmc_priv(mmc);
	uint32_t reg;

	if (priv->debug)
		dev_err(&priv->pdev->dev,
			"get_cd\n");

	q5_sd_read(priv,QSD_STATUS,&reg);
	return GETREG(reg,CARD_DETECTED);
}

static int q5_sd_wait_dma(struct q5_sd_priv *priv, int direction){
	uint32_t status;
	uint32_t dma_status;
	unsigned long expiration=jiffies+SD_TIMEOUT;


	while (1){
		q5_sd_read(priv,QSD_DMA_STATUS,&dma_status);

		if ((GETREG(dma_status,DMA_IDLE)==1)){
			if ((GETREG(dma_status,DMA_TX_ERR)==1)||
					(GETREG(dma_status,DMA_RX_ERR)==1)){
				dev_err(&priv->pdev->dev, "DATA Dma CRC Error Status=0x%.8x\n",dma_status);
				return -ETIMEDOUT;
				//return -EILSEQ;
			}
			if (direction==DMA_FROM_DEVICE)
				return 0;


			q5_sd_read(priv,QSD_STATUS,&status);
			if (GETREG(status,DATA_TX_STATUS)==DATA_TX_OK)
				return 0;

			dev_err(&priv->pdev->dev, "DATA CRC Error Status=0x%.8x\n",status);
				return -EILSEQ;
		}

		if (time_before(expiration,jiffies)){
			dev_err(&priv->pdev->dev, "DATA Timeout 1 DMAStatus=0x%.8x\n",dma_status);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int q5_sd_wait_tx_idle(struct q5_sd_priv *priv){
	uint32_t status;
	unsigned long expiration=jiffies+SD_TIMEOUT;

	while (1){
		q5_sd_read(priv,QSD_STATUS,&status);
		if (GETREG(status,CMD_TX_IDLE)==1)
			break;

		if (time_before(expiration,jiffies)){
			dev_err(&priv->pdev->dev, "CMD TX IDLE Timeout Status=0x%.8x\n",status);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int q5_sd_wait_idle(struct q5_sd_priv *priv){
	uint32_t status;
	unsigned long expiration=jiffies+SD_TIMEOUT;

	while (1){
		q5_sd_read(priv,QSD_STATUS,&status);
		if (GETREG(status,CMD_IDLE)==1)
			break;

		if (time_before(expiration,jiffies)){
			dev_err(&priv->pdev->dev, "CMD IDLE Timeout Status=0x%.8x\n",status);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int q5_sd_setup_dma(struct q5_sd_priv *priv, struct mmc_request *req){
	uint32_t data_ctrl,dma_ctrl;

	if (priv->debug)
		dev_err(&priv->pdev->dev, "request DATA blocks=%d blksz=%d\n",req->data->blocks, req->data->blksz);

	if (req->data->blksz&3){
		dev_err(&priv->pdev->dev, "Blocksize must be multiple of 4\n");
		return -EINVAL;
	}

	if (sg_phys(req->data->sg)&3){
		dev_err(&priv->pdev->dev, "DMA address must be word aligned\n");
		return -EINVAL;
	}

	if (req->data->blocks>MAX_BLK_COUNT){
		dev_err(&priv->pdev->dev, "Too many blocks to write\n");
		return -EINVAL;
	}

	if (req->data->blksz>MAX_BLK_SIZE){
		dev_err(&priv->pdev->dev, "Blocksize too big\n");
		return -EINVAL;
	}

	//dma address
	q5_sd_write(priv,QSD_DMA_ADDR,sg_phys(req->data->sg));

	//reset data_ctrl
	q5_sd_read(priv,QSD_DATA_CONTROL,&data_ctrl);
	data_ctrl=SETREG(data_ctrl,DATA_RX_ENABLE,0);
	data_ctrl=SETREG(data_ctrl,DATA_TX_ENABLE,0);
	data_ctrl=SETREG(data_ctrl,DATA_RX_RST,1);
	data_ctrl=SETREG(data_ctrl,DATA_RECV_LEN,0);
	q5_sd_write(priv,QSD_DATA_CONTROL,data_ctrl);

	//continue
	data_ctrl=SETREG(data_ctrl,DATA_RX_RST,0);
	data_ctrl=SETREG(data_ctrl,DATA_RECV_LEN,req->data->blksz-1);

	dma_ctrl=SETREG(0,DMA_ENABLE,1);
	dma_ctrl=SETREG(dma_ctrl,DMA_IRQ_ENA,1);

	if (req->data->flags & MMC_DATA_WRITE){
		dma_map_sg(&priv->pdev->dev,req->data->sg,req->data->sg_len,DMA_TO_DEVICE);
		data_ctrl=SETREG(data_ctrl,DATA_TX_ENABLE,1);
		dma_ctrl=SETREG(dma_ctrl,DMA_DIR,1);
	}
	else{
		dma_map_sg(&priv->pdev->dev,req->data->sg,req->data->sg_len,DMA_FROM_DEVICE);
		data_ctrl=SETREG(data_ctrl,DATA_RX_ENABLE,1);
	}
	//Data ctrl
	q5_sd_write(priv,QSD_DATA_CONTROL,data_ctrl);

	//Dma ctrl
	q5_sd_write(priv,QSD_DMA_CONTROL,dma_ctrl);

	if (req->data->flags & MMC_DATA_READ){
		//Trigger dma
		unsigned long timeout;
		timeout=req->data->timeout_ns/1000000;//msec
		timeout*=req->data->blocks;
		timeout*=HZ/1000; //msec
		timeout+=jiffies+SD_TIMEOUT;
		mod_timer(&priv->timeout_dma_timer,timeout);
		q5_sd_write(priv,QSD_DMA_LENGTH,req->data->blocks);
	}

	return 0;
}

static int q5_sd_send_cmd(struct q5_sd_priv *priv, struct mmc_command *cmd, int do_irq, int do_wait){
	uint32_t cmd_ctrl;
	int ret;

	q5_sd_read(priv,QSD_CMD_CONTROL,&cmd_ctrl);
	cmd_ctrl=SETREG(cmd_ctrl,CMD_ENABLE,1);

	//Comand specifics
	if (do_irq)
		cmd_ctrl=SETREG(cmd_ctrl,CMD_IRQ_ENA,1);
	else
		cmd_ctrl=SETREG(cmd_ctrl,CMD_IRQ_ENA,0);

	if (do_wait)
		cmd_ctrl=SETREG(cmd_ctrl,WAIT_READY,1);
	else
		cmd_ctrl=SETREG(cmd_ctrl,WAIT_READY,0);

	if (cmd->flags & MMC_RSP_136){
		cmd_ctrl=SETREG(cmd_ctrl,RECV_LEN,17);
		cmd_ctrl=SETREG(cmd_ctrl,SKIP_CRC,1);
	}
	else if (!(cmd->flags & MMC_RSP_PRESENT)){
		cmd_ctrl=SETREG(cmd_ctrl,RECV_LEN,0);
		cmd_ctrl=SETREG(cmd_ctrl,SKIP_CRC,0);
	}
	else{
		cmd_ctrl=SETREG(cmd_ctrl,RECV_LEN,6);
		cmd_ctrl=SETREG(cmd_ctrl,SKIP_CRC,0);
	}

	//Prepare timeout
	if (do_irq)
		mod_timer(&priv->timeout_cmd_timer,jiffies+SD_TIMEOUT);

	//Write control
	q5_sd_write(priv,QSD_CMD_CONTROL,cmd_ctrl);

	//Write command
	ret=q5_sd_write_cmd(priv,cmd->opcode|SD_HOST_CMD);
	ret|=q5_sd_write_cmd(priv,(cmd->arg>>24)&0xff);
	ret|=q5_sd_write_cmd(priv,(cmd->arg>>16)&0xff);
	ret|=q5_sd_write_cmd(priv,(cmd->arg>>8)&0xff);
	ret|=q5_sd_write_cmd(priv,cmd->arg&0xff);

	return ret;
}

static int q5_sd_parse_resp(struct q5_sd_priv *priv, struct mmc_command *cmd){
	uint32_t cmd_ctrl,status;
	uint8_t dummy;
	uint8_t cmd_rsp;
	int i;
	int ret=0;
	int res_len=6;

	//Wait tx idle
	ret=q5_sd_wait_tx_idle(priv);
	if(ret){
		dev_err(&priv->pdev->dev,"CMD%d: wait TX idle timeout\n",cmd->opcode);
		goto end_cmd;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		goto no_data;

	if (cmd->flags & MMC_RSP_136)
		res_len=17;

	/*Ignore first byte in long resp... this is done by the core
		ret=q5_sd_read_cmd(priv,&dummy);
		if (ret){
			dev_err(&priv->pdev->dev,"CMD%d: read dummy head timeout\n",cmd->opcode);
			goto end_cmd;
		}
	}*/

	//Read cmd rsp
	for(i=res_len-2;i>=0;i--){

		ret=q5_sd_read_cmd(priv,&cmd_rsp);
		if (ret){
			dev_err(&priv->pdev->dev,"CMD%d: read resp %d timeout\n",cmd->opcode,res_len-2-i);
			goto end_cmd;
		}

		cmd->resp[3-(i/4)]<<=8;
		cmd->resp[3-(i/4)]|=cmd_rsp&0xff;
	}

	//Ignore last byte in short resp && invert words
	if (!(cmd->flags & MMC_RSP_136)){
		cmd->resp[0]=cmd->resp[3];
		cmd->resp[1]=cmd->resp[2];
		ret=q5_sd_read_cmd(priv,&dummy);
		if (ret){
			dev_err(&priv->pdev->dev,"CMD%d:read dummy tail timeout\n",cmd->opcode);
			goto end_cmd;
		}
	}

	if (priv->debug){
		dev_err(&priv->pdev->dev,
				"res_len=%d\n",res_len);
		for(i=0;i<4;i++){
			dev_err(&priv->pdev->dev,
					"resp[%d]=0x%.8x\n",i,cmd->resp[i]);
		}
	}

no_data:
	//Wait for Idle
	ret=q5_sd_wait_idle(priv);
	if(ret){
		dev_err(&priv->pdev->dev,"CMD%d: wait idle timeout\n",cmd->opcode);
		goto end_cmd;
	}

	//Check crc
	if (cmd->flags & MMC_RSP_CRC){
		q5_sd_read(priv,QSD_STATUS,&status);
		if (GETREG(status,CMD_RX_CRC)==1){
			dev_err(&priv->pdev->dev, "CMD%d: CRC Error!!! 0x%.2x\n",cmd->opcode,status);
			ret=-EILSEQ;
		}
	}

	//Disable ctrl iface
end_cmd:
	q5_sd_read(priv,QSD_CMD_CONTROL,&cmd_ctrl);
	cmd_ctrl=SETREG(cmd_ctrl,CMD_ENABLE,0);
	q5_sd_write(priv,QSD_CMD_CONTROL,cmd_ctrl);
	return ret;
}

static int q5_sd_finish_dma(struct q5_sd_priv *priv, struct mmc_request *req){
	int len=req->data->blocks * req->data->blksz;
	int direction;
	uint32_t data_ctrl;
	int ret=0;

	if (req->data->flags & MMC_DATA_WRITE)
		direction=DMA_TO_DEVICE;
	else direction=DMA_FROM_DEVICE;

	/*Wait for dma*/
	ret=q5_sd_wait_dma(priv,direction);
	if (ret)
		len=0;

	/*Unmap SG*/
	dma_unmap_sg(&priv->pdev->dev,req->data->sg,req->data->sg_len,direction);

	/*Disable data*/
	q5_sd_read(priv,QSD_DATA_CONTROL,&data_ctrl);
	data_ctrl=SETREG(data_ctrl,DATA_RX_ENABLE,0);
	data_ctrl=SETREG(data_ctrl,DATA_TX_ENABLE,0);
	data_ctrl=SETREG(data_ctrl,DATA_RX_RST,1);
	q5_sd_write(priv,QSD_DATA_CONTROL,data_ctrl);

	/*Stop dma*/
	q5_sd_write(priv,QSD_DMA_CONTROL,0);

	/*Save bytes xfered*/
	req->data->bytes_xfered=len;

	if (priv->debug){
		uint8_t *pdata;
		int i;
		dev_err(&priv->pdev->dev, "CMD%d DATA len=%d ret=%d\n",req->cmd->opcode,len,ret);
		pdata=sg_virt(req->data->sg);
		for (i=0;i<len;i++){
			dev_err(&priv->pdev->dev, "DATA[%3d]=0x%.2x (%c)\n",i,pdata[i],pdata[i]);
		}
	}

	return ret;
}

static void q5_sd_work_cmd_completed(struct work_struct *work);

static void q5_sd_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct q5_sd_priv *priv=mmc_priv(mmc);
	int ret=-EINVAL;
	int do_irq=0;
	int do_dma=0;
	int do_wait=0;


	mutex_lock(&priv->req_mutex);

	if((req->cmd->opcode==5)||(req->cmd->opcode==52)){
		dev_err(&priv->pdev->dev,"Ignoring CMD%d\n",req->cmd->opcode);
		ret=-EINVAL;
		goto sd_req_error;
	}

	if (priv->req){
		dev_err(&priv->pdev->dev,
				"Already in other req. Aborting\n");
		ret=-ETIMEDOUT;
		goto sd_req_error;
	}

	if ((req->cmd->flags & MMC_RSP_BUSY)||(req->data))
		do_irq=1;

	if (req->cmd->data)
		do_dma=1;

	if (req->cmd->flags & MMC_RSP_BUSY)
		do_wait=1;

	if (priv->debug)
		dev_err(&priv->pdev->dev,
				"request cmd%d arg=0x%.8x irq=%d dma=%d\n",req->cmd->opcode,req->cmd->arg,do_irq,do_dma);

	if (do_dma){
		if (req->data->sg_len!=1){
			dev_err(&priv->pdev->dev,
					"SG not implemented\n");
			goto sd_req_error;
		}
		ret=q5_sd_setup_dma(priv,req);
		if(ret){
			goto sd_req_error;
		}
	}

	ret=q5_sd_send_cmd(priv,req->cmd,do_irq,do_wait);
	if(ret){
		goto sd_req_error;
	}

	//Save request
	priv->req=req;
	priv->mode=IN_CMD;
	mutex_unlock(&priv->req_mutex);

	//Irq will handle it
	if (do_irq)
		return;

	//Non irq mode
	q5_sd_work_cmd_completed(&priv->cmd_completed);
	return;

sd_req_error:
	priv->mode=IDLE;
	priv->req=NULL;
	req->cmd->error=ret;
	mutex_unlock(&priv->req_mutex);
	if (priv->debug)
		dev_err(&priv->pdev->dev, "mmc_req_done 1: cmd%d ret=%d\n",req->cmd->opcode,req->cmd->error);
	mmc_request_done(mmc,req);
	return;
}

static unsigned int q5_sd_calc_div(struct q5_sd_priv *priv,int req_value){
	unsigned int div;

	if (priv->debug)
		dev_err(&priv->pdev->dev,
			"clk=%d\n",req_value);

	if (req_value==0)
		return MAX_FREQ_DIV;

	div=DIV_ROUND_UP(priv->freq,req_value);

	if (div<MIN_FREQ_DIV)
		div=MIN_FREQ_DIV;

	if (div>MAX_FREQ_DIV)
		div=MAX_FREQ_DIV;

	return div;
}

static void q5_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct q5_sd_priv *priv=mmc_priv(mmc);
	uint32_t data_ctrl=0;
	uint32_t cmd_if_ctrl=0;

	if (priv->debug)
		dev_err(&priv->pdev->dev,
			"set_ios\n");

	switch (ios->power_mode){
		case MMC_POWER_OFF:
			if (priv->debug)
				dev_err(&priv->pdev->dev,
					"power off\n");
			data_ctrl=SETREG(data_ctrl,DATA_RX_RST,1);
			break;
		case MMC_POWER_ON:
			if (priv->debug)
				dev_err(&priv->pdev->dev,
					"power on\n");
			cmd_if_ctrl=SETREG(cmd_if_ctrl,POWER_ON,1);
			break;
		case MMC_POWER_UP:
			if (priv->debug)
				dev_err(&priv->pdev->dev,
					"power up\n");
			cmd_if_ctrl=SETREG(cmd_if_ctrl,POWER_ON,1);
			data_ctrl=SETREG(data_ctrl,DATA_RX_RST,1);
			break;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4){
		if (priv->debug)
			dev_err(&priv->pdev->dev,
				"4 bits\n");
		data_ctrl=SETREG(data_ctrl,MODE_4BIT,1);
	}
	else
		data_ctrl=SETREG(data_ctrl,MODE_4BIT,0);

	cmd_if_ctrl=SETREG(cmd_if_ctrl,CLOCK_DIVIDE,q5_sd_calc_div(priv,ios->clock));

	q5_sd_write(priv,QSD_CMD_IF_CONTROL,cmd_if_ctrl);
	q5_sd_write(priv,QSD_DATA_CONTROL,data_ctrl);

	return;
}

static void q5_sd_work_dma_completed(struct work_struct *work){
	struct q5_sd_priv *priv=  container_of(work,struct q5_sd_priv,dma_completed);
	struct mmc_request *req;
	int ret=0;

	mutex_lock(&priv->req_mutex);

	if (priv->mode==IN_CMD){
		dev_err(&priv->pdev->dev,"CMD%d dma before cmd\n",priv->req->cmd->opcode);
		mutex_unlock(&priv->req_mutex);
		return;
	}

	if (priv->mode!=IN_DMA){
		dev_err(&priv->pdev->dev,
				"dma_complete: Wrong priv->mode(%d), timeout too fast or spureous IRQ\n",priv->mode);
		mutex_unlock(&priv->req_mutex);
		return;
	}

	req=priv->req;
	if (!req){
		dev_err(&priv->pdev->dev,
				"dma_complete: Request is NULL, timeout too fast or spureous IRQ...\n");
		mutex_unlock(&priv->req_mutex);
		return;
	}

	if(!req->data){
		dev_err(&priv->pdev->dev,
				"dma_complete: DATA is NULL, timeout too fast or spureous IRQ...\n");
		mutex_unlock(&priv->req_mutex);
		return;
	}

	if (priv->req->cmd->opcode==13)
		msleep(SD_TIMEOUT_MS);
	if (priv->req->cmd->opcode==6)
		msleep(SD_TIMEOUT_MS);
	req->data->error=q5_sd_finish_dma(priv,req);

	if (priv->debug)
		if (req->stop)
			dev_err(&priv->pdev->dev, "dma: REQ STOP Command :S\n");

	if (req->data->stop){
		if (priv->debug)
			dev_err(&priv->pdev->dev, "dma: DATA STOP Command :S\n");
		priv->mode=IN_STOP;

		ret=q5_sd_send_cmd(priv,req->data->stop,1,1);
		if (ret){
			req->data->stop->error=ret;
			goto end_dma;
		}
		mutex_unlock(&priv->req_mutex);
		return;
	}


end_dma:
	priv->mode=IDLE;
	priv->req=NULL;
	mutex_unlock(&priv->req_mutex);
	if (priv->debug)
		dev_err(&priv->pdev->dev, "mmc_req_done 2: cmd%d ret=%d\n",req->cmd->opcode,req->cmd->error);
	mmc_request_done(priv->mmc_host,req);

	return ;
}

static void q5_sd_timeout_dma_timer(long unsigned int data){
	struct q5_sd_priv *priv= (struct q5_sd_priv *)data;
	struct mmc_request *req=priv->req;

	if (!req)
		dev_err(&priv->pdev->dev, "timer dma: Request is NULL, timeout too fast...\n");
	else{
		dev_err(&priv->pdev->dev, "timer dma: Timeout on CMD%d, force read\n",req->cmd->opcode);
		schedule_work(&priv->dma_completed);
	}

	return;
}

static void q5_sd_work_cmd_completed(struct work_struct *work){
	struct q5_sd_priv *priv=  container_of(work,struct q5_sd_priv,cmd_completed);
	struct mmc_request *req;
	int ret=0;

	mutex_lock(&priv->req_mutex);

	if ((priv->mode!=IN_STOP)&&(priv->mode!=IN_CMD)){
		dev_err(&priv->pdev->dev,
				"cmd_complete: Wrong priv->mode(%d), timeout too fast or spureous IRQ\n",priv->mode);
		mutex_unlock(&priv->req_mutex);
		return;
	}

	req=priv->req;
	if (!req){
		dev_err(&priv->pdev->dev,
				"cmd_complete: Request is NULL, timeout too fast or spureous IRQ...\n");
		mutex_unlock(&priv->req_mutex);
		return;
	}

	if (priv->mode==IN_STOP){
		ret=q5_sd_parse_resp(priv,req->data->stop);
		req->data->stop->error=ret;
	}
	else{
		ret=q5_sd_parse_resp(priv,req->cmd);
		req->cmd->error=ret;
	}

	if(ret)
		goto end_cmd_complete;

	if ((priv->mode==IN_CMD)&&(req->data)){
		priv->mode=IN_DMA;

		if (req->data->flags & MMC_DATA_WRITE){
			//Trigger dma
			unsigned long timeout;
			timeout=req->data->timeout_ns/1000000;//msec
			timeout*=req->data->blocks;
			timeout*=HZ/1000; //msec
			timeout+=jiffies+HZ;
			mod_timer(&priv->timeout_dma_timer,timeout);
			q5_sd_write(priv,QSD_DMA_LENGTH,req->data->blocks);
		}
		mutex_unlock(&priv->req_mutex);

		return;
	}

end_cmd_complete:
	priv->mode=IDLE;
	priv->req=NULL;
	req->cmd->error=ret;
	mutex_unlock(&priv->req_mutex);

	if (priv->debug){
		if (req->stop)
			dev_err(&priv->pdev->dev, "cmd: REQ STOP Command :S\n");
		dev_err(&priv->pdev->dev, "mmc_req_done 3: cmd%d ret=%d\n",req->cmd->opcode,req->cmd->error);
	}
	mmc_request_done(priv->mmc_host,req);

	return;
}

static void q5_sd_timeout_cmd_timer(long unsigned int data){
	struct q5_sd_priv *priv= (struct q5_sd_priv *)data;
	struct mmc_request *req=priv->req;

	if (!req)
		dev_err(&priv->pdev->dev, "timer cmd: Request is NULL, timeout too fast...\n");
	else{
		dev_err(&priv->pdev->dev, "timer cmd: Timeout on CMD%d, force read\n",req->cmd->opcode);
		schedule_work(&priv->cmd_completed);
	}

	return;
}

static irqreturn_t q5_sd_irq_handler (int irq, void *data){
	struct q5_sd_priv *priv=data;
	uint32_t status;
	uint32_t dma_status;

	//CMD irq
	q5_sd_read(priv,QSD_STATUS,&status);
	if (GETREG(status,CMD_IRQ_STATUS)){
		if (priv->debug)
			dev_err(&priv->pdev->dev, "cmd irq\n");
		del_timer(&priv->timeout_cmd_timer);
		schedule_work(&priv->cmd_completed);

		//ack
		status=SETREG(0,CMD_IRQ_STATUS,1);
		q5_sd_write(priv,QSD_STATUS,status);
		return IRQ_HANDLED;
	}

	//DMA irq
	q5_sd_read(priv,QSD_DMA_STATUS,&dma_status);
	if (GETREG(dma_status,DMA_IRQ_STATUS)){
		if (priv->debug)
			dev_err(&priv->pdev->dev, "dma irq\n");
		del_timer(&priv->timeout_dma_timer);
		schedule_work(&priv->dma_completed);

		//ack
		dma_status=SETREG(0,DMA_IRQ_STATUS,1);
		q5_sd_write(priv,QSD_DMA_STATUS,dma_status);
		return IRQ_HANDLED;
	}


	//Not my irq
	return 0;
}

static const struct mmc_host_ops q5_sd_ops = {
	.request = q5_sd_request,
	.set_ios = q5_sd_set_ios,
	.get_cd = q5_sd_get_cd,
	.get_ro = q5_sd_get_ro,
};

static int q5_sd_of_probe(struct platform_device *pdev)
{
	struct resource mem;
	void * iomem;
	struct q5_sd_priv *priv;
	struct mmc_host *mmc;
	const uint32_t *base_freq;
	int size;
	int irq;

	/*Get pdev resources*/
	if (of_address_to_resource(pdev->dev.of_node,0,&mem)){
		dev_err(&pdev->dev, "Unable to get memory address\n");
		goto of_mem_notfound;
	}

	irq=platform_get_irq(pdev, 0);
	if (irq<0){
		dev_err(&pdev->dev, "Unable to get irq\n");
		goto of_irq_notfound;
	}
	base_freq=of_get_property(pdev->dev.of_node,"base-freq",&size);
	if((size!=sizeof(uint32_t))||(!base_freq)||(*base_freq==0)){
		dev_err(&pdev->dev, "Unable to get base_freq\n");
		goto base_freq_notfound;
	}

	/*request memory*/
	if (!request_mem_region(mem.start,
				resource_size(&mem), DRIVER_NAME)){
		dev_err(&pdev->dev, "Unable to request memory address\n");
		goto request_mem_failed;
	}

	iomem = devm_ioremap(&pdev->dev, mem.start, resource_size(&mem));
	if (!iomem){
		dev_err(&pdev->dev, "Unable to ioremap memory\n");
		goto ioremap_failed;
	}

	/*MMC host alloc*/
	mmc=mmc_alloc_host(sizeof(*priv),&pdev->dev);
	if(!mmc){
		goto mmc_no_mem;
	}

	/*Fill priv structure*/
	priv=mmc_priv(mmc);
	priv->mmc_host=mmc;
	priv->irq=irq;
	priv->mem=mem;
	priv->iomem=iomem;
	priv->freq=be32_to_cpu(*base_freq);
	priv->req=NULL;
	priv->debug=0;
	priv->mode=IDLE;
	priv->pdev=pdev;

	/*Fill mmc structure*/
	mmc->f_min=(priv->freq/MAX_FREQ_DIV);
	if (priv->freq%MAX_FREQ_DIV)
		mmc->f_min++;
	mmc->f_max=priv->freq/MIN_FREQ_DIV;
	mmc->ocr_avail=MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	mmc->caps |= MMC_CAP_SD_HIGHSPEED;
	mmc->caps |= MMC_CAP_NEEDS_POLL;
	mmc->caps |= MMC_CAP_POWER_OFF_CARD;
	mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;
	mmc->ops = &q5_sd_ops;
	mmc->max_segs = 1;
	mmc->max_blk_size = MAX_BLK_SIZE;
	mmc->max_blk_count = MAX_BLK_COUNT;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_seg_size;

	dev_set_drvdata(&pdev->dev,mmc);

	/*Mutex*/
	mutex_init(&priv->req_mutex);

	/*Tasklets*/
	INIT_WORK(&priv->cmd_completed,q5_sd_work_cmd_completed);
	INIT_WORK(&priv->dma_completed,q5_sd_work_dma_completed);
	setup_timer(&priv->timeout_cmd_timer,q5_sd_timeout_cmd_timer,(unsigned long)priv);
	setup_timer(&priv->timeout_dma_timer,q5_sd_timeout_dma_timer,(unsigned long)priv);

	/*init host*/
	mmc_add_host(mmc);

	/*Irq*/
	if (request_irq(irq,q5_sd_irq_handler,IRQF_SHARED,DRIVER_NAME,priv)){
		dev_err(&pdev->dev, "Unable to request data IRQ\n");
		goto req_irq;
	}

	dev_info(&pdev->dev, "Qtec SD core at 0x%.8x\n",(uint32_t)mem.start);

	return 0;

	free_irq(irq,&priv);
req_irq:
	del_timer(&priv->timeout_dma_timer);
	del_timer(&priv->timeout_cmd_timer);
	mmc_remove_host(mmc);
	mmc_free_host(mmc);
mmc_no_mem:
	devm_iounmap(&pdev->dev,iomem);
ioremap_failed:
	release_mem_region(mem.start,
			resource_size(&mem));
request_mem_failed:
base_freq_notfound:
of_irq_notfound:
of_mem_notfound:

	return -EIO;
}

static int q5_sd_of_remove(struct platform_device *pdev){
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct q5_sd_priv *priv=mmc_priv(mmc);

	mmc_remove_host(mmc);
	mmc_free_host(mmc);
	del_timer(&priv->timeout_cmd_timer);
	del_timer(&priv->timeout_dma_timer);
	free_irq(priv->irq,priv);

	release_mem_region(priv->mem.start,
			resource_size(&priv->mem));
	return 0;
}

static struct of_device_id q5_sd_of_match[] = {
	{ .compatible = "qtec,xps_sd-if-1.00.a",},
	{ /* EOL */}
};

MODULE_DEVICE_TABLE(of, q5_sd_of_match);

static struct platform_driver q5_sd_of_driver = {
	.probe		= q5_sd_of_probe,
	.remove		= q5_sd_of_remove,
	.driver		={
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table	= q5_sd_of_match,
	},
};

static int __init q5_sd_init(void)
{
	return platform_driver_register(&q5_sd_of_driver);
}

static void __exit q5_sd_exit(void)
{
	platform_driver_unregister(&q5_sd_of_driver);
}

module_init(q5_sd_init);
module_exit(q5_sd_exit);

MODULE_AUTHOR("Ricardo Ribalda <ricardo.ribalda@gmail.com>");
MODULE_DESCRIPTION("Qtec SD driver");
MODULE_LICENSE("GPL");
