#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/mii.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <linux/types.h>
#include <netinet/in.h>

void help(void)
{
	printf("CVITEK ethernet phy reader/writer Usage:\n");
	printf("smi read operation: ephy -s reg_addr\n");
	printf("smi write operation: ephy -s reg_addr value\n");
	printf("mdio read operation: ephy -m dev_addr reg_addr\n");
	printf("mdio write operation: ephy -m dev_addr reg_addr value\n");
	printf("For example:\n");
	printf("read smi phy register 1: ephy -s 1\n");
	printf("write smi phy register 0 with 0x12: ephy -s 0 0x12\n\n");
	printf("read mdio phy register 1: ephy -m 0x7 0x3c\n");
	printf("write mdio phy register 0 with 0x12: ephy -m 0x7 0x3c 0x11\n\n");
	exit(0);
}
int sockfd;

int main(int argc, char *argv[])
{

	if (argc < 2 || !strcmp(argv[1], "-h"))
		help();

	struct mii_ioctl_data *mii = NULL;
	struct ifreq ifr;
	int ret;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);

	sockfd = socket(PF_LOCAL, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		printf("Create sockfd failed\n");
		return -1;
	}

	//get phy address in smi bus
	ret = ioctl(sockfd, SIOCGMIIPHY, &ifr);
	if (ret < 0) {
		printf("Get phy address error\n");
		return -1;
	}

	mii = (struct mii_ioctl_data *)&ifr.ifr_data;

	if (!strcmp(argv[1], "-s")) {
		switch (argc) {
		case 3:
			mii->reg_num = (uint16_t)strtoul(argv[2], NULL, 0);
			ret = ioctl(sockfd, SIOCGMIIREG, &ifr);
			if (ret < 0) {
				printf("smi read error\n");
				return -1;
			}
			printf("Smi read reg: 0x%x  value: 0x%04x\n\n", mii->reg_num, mii->val_out);
			break;
		case 4:
			mii->reg_num = (uint16_t)strtoul(argv[2], NULL, 0);
			mii->val_in = (uint16_t)strtoul(argv[3], NULL, 0);

			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
				printf("smi write error\n");
				return -1;
			}
			printf("Smi write reg: 0x%x  value: 0x%04x\n\n", mii->reg_num, mii->val_in);
			break;
		default:
			help();
			break;
		}
	} else if (!strcmp(argv[1], "-m")) {
		switch (argc) {
		case 4: /* read */
			mii->reg_num = 0xd;
			mii->val_in = (uint16_t)strtoul(argv[2], NULL, 0);
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
			printf("mdio write device addr error\n");
			return -1;
			}

			mii->reg_num = 0xe;
			mii->val_in = (uint16_t)strtoul(argv[3], NULL, 0);
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
				printf("mdio write register addr error\n");
				return -1;
			}

			mii->reg_num = 0xd;
			mii->val_in = (0x1 << 14 | (uint16_t)strtoul(argv[2], NULL, 0));
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
				printf("mdio write register addr error\n");
				return -1;
			}

			mii->reg_num = 0xe;
			ret = ioctl(sockfd, SIOCGMIIREG, &ifr);
			if (ret < 0) {
				printf("mdio read register value error\n");
				return -1;
			}
			printf("Mdio read dev: 0x%x  reg: 0x%x  value: 0x%04x\n\n", (uint16_t)strtoul(argv[2], NULL, 0),
			(uint16_t)strtoul(argv[3], NULL, 0), mii->val_out);
			break;
		case 5:
			mii->reg_num = 0xd;
			mii->val_in = (uint16_t)strtoul(argv[2], NULL, 0);
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
			printf("mdio write device addr error\n");
			return -1;
			}

			mii->reg_num = 0xe;
			mii->val_in = (uint16_t)strtoul(argv[3], NULL, 0);
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
			printf("mdio write register addr error\n");
			return -1;
			}

			mii->reg_num = 0xd;
			mii->val_in = (0x1 << 14 | (uint16_t)strtoul(argv[2], NULL, 0));
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
				printf("mdio write register addr error\n");
			return -1;
			}

			mii->reg_num = 0xe;
			mii->val_in = (uint16_t)strtoul(argv[4], NULL, 0);
			ret = ioctl(sockfd, SIOCSMIIREG, &ifr);
			if (ret < 0) {
				printf("mdio read register value error\n");
				return -1;
			}
			printf("Mdio Write dev: 0x%x reg: 0x%x  value: 0x%04x\n\n", (uint16_t)strtoul(argv[2], NULL, 0),
			(uint16_t)strtoul(argv[3], NULL, 0), mii->val_in);
			break;
		default:
			help();
			break;
		}
	}
	close(sockfd);
}
