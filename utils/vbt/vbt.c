#include <lil/vbt.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static struct option longopts[] = {
	{"blocks", no_argument, 0, 'b'},
	{0, 0, 0, 0},
};

int main(int argc, char *argv[]) {
	bool print_blocks = false;

	while(1) {
		int option_index = 0;
		int c = getopt_long(argc, argv, "b", longopts, &option_index);

		if(c == -1) {
			break;
		}

		switch(c) {
			case 'b': {
				print_blocks = true;
				break;
			}
			default: {
				fprintf(stderr, "getopt_long returned %c\n", c);
				break;
			}
		}
	}

	if((optind + 1) != argc) {
		fprintf(stderr, "no file given\n");
		exit(1);
	}

	const char *filepath = argv[optind];
	int fd = open(filepath, O_RDONLY);
	if(fd < 0) {
		fprintf(stderr, "could not open file '%s': %s\n", argv[1], strerror(errno));
		exit(1);
	}

	struct stat statbuf;
	int err = fstat(fd, &statbuf);
	assert(err >= 0);

	void *vbt = mmap(NULL, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	assert(vbt != MAP_FAILED);
	close(fd);

	const struct vbt_header *hdr = vbt_get_header(vbt, statbuf.st_size);
	assert(hdr);

	printf("VBT header version %u.%u\n", hdr->version / 100, hdr->version % 100);

	const struct bdb_header *bdb_hdr = vbt_get_bdb_header(hdr);
	assert(bdb_hdr);

	if(print_blocks) {
		printf("Blocks:");
		for(size_t i = 0; i < 256; i++) {
			if(vbt_get_bdb_block(hdr, i)) {
				printf(" %zu", i);
			}
		}
		printf("\n");
	}

	munmap(vbt, statbuf.st_size);

	return 0;
}
