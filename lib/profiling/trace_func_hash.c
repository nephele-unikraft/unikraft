#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <sys/mman.h>
#include <elf.h>
#define HASHTABLE_DONT_FREE_KEY 1
#include <hashtable.h>
#include "profile.h"

#define DEBUG_ELF_PARSER 0
#if DEBUG_ELF_PARSER
#define DEBUG(fmt, ...)     fprintf(stderr, fmt, ## __VA_ARGS__)
#else
#define DEBUG(fmt, ...)
#endif


static struct hashtable *func_map;

__noinstrument int func_map_init(const char *filename);
__noinstrument const char *func_map_get(unsigned long address);

static __noinstrument
int map_elf_file(const char *filename, void **mapaddr, long *mapsize)
{
    FILE *f;
    long fsize;
    void *fbytes;
    int rc;

    f = fopen(filename, "r");
    if (f == NULL) {
        perror("fopen()");
        rc = -ENOENT;
        goto out;
    }

    rc = fseek(f, 0, SEEK_END);
    if (rc) {
        perror("fseek()");
        fclose(f);
        goto out;
    }
    fsize = ftell(f);

    fbytes = mmap(NULL, fsize, PROT_READ, MAP_PRIVATE, fileno(f), 0);
    if (fbytes == NULL) {
        fclose(f);
        perror("mmap()");
        rc = errno;
        goto out;
    }
    fclose(f);

    *mapaddr = fbytes;
    *mapsize = fsize;

out:
    return rc;
}

static __noinstrument
int parse_symtable(Elf64_Sym *symtable, char *strtable, size_t table_sz)
{
    Elf64_Sym *sym;
    char *func_name;
    int rc = 0;

    for (size_t j = 0; j * sizeof(Elf64_Sym) < table_sz; j++) {
        sym = symtable + j;

        if ( ELF64_ST_TYPE(sym->st_info) != STT_FUNC )
            continue;
        if ( sym->st_name == 0 )
            continue;
        if ( sym->st_value == 0 )
            continue;

        func_name = strtable + sym->st_name;

#if TEST_ELF_PARSER
#if 0
        printf("SYMBOL TABLE ENTRY %zd\n", j);
        printf("st_name = %d (%s)\n", sym->st_name, strtable + sym->st_name);
        printf("st_info = %d\n", sym->st_info);
        printf("st_other = %d\n", sym->st_other);
        printf("st_shndx = %d\n", sym->st_shndx);
        printf("st_value = %p\n", (void *) sym->st_value);
        printf("st_size = %zd\n", sym->st_size);
        printf("\n");
#else

#if 0
        printf("%lx;%s\n", sym->st_value, strtable + sym->st_name);
#else
        printf("%s\n", func_name);
#endif
#endif
#endif

        rc = hashtable_insert(func_map, (void *) sym->st_value, strdup(func_name));
        if (rc)
            rc = 0;
        else {
            rc = -1;
            break;
        }
    }

    return rc;
}

#if DEBUG_ELF_PARSER
static
const char *sht_string(unsigned long val)
{
    switch (val) {
    case SHT_NULL: return "SHT_NULL";
    case SHT_PROGBITS: return "SHT_PROGBITS";
    case SHT_SYMTAB: return "SHT_SYMTAB";
    case SHT_STRTAB: return "SHT_STRTAB";
    case SHT_RELA: return "SHT_RELA";
    case SHT_HASH: return "SHT_HASH";
    case SHT_DYNAMIC: return "SHT_DYNAMIC";
    case SHT_NOTE: return "SHT_NOTE";
    case SHT_NOBITS: return "SHT_NOBITS";
    case SHT_REL: return "SHT_REL";
    case SHT_SHLIB: return "SHT_SHLIB";
    case SHT_DYNSYM: return "SHT_DYNSYM";
    case SHT_INIT_ARRAY: return "SHT_INIT_ARRAY";
    case SHT_FINI_ARRAY: return "SHT_FINI_ARRAY";
    case SHT_PREINIT_ARRAY: return "SHT_PREINIT_ARRAY";
    case SHT_GROUP: return "SHT_GROUP";
    case SHT_SYMTAB_SHNDX: return "SHT_SYMTAB_SHNDX";
    case SHT_NUM: return "SHT_NUM";
    case SHT_LOOS: return "SHT_LOOS";
    case SHT_GNU_ATTRIBUTES: return "SHT_GNU_ATTRIBUTES";
    case SHT_GNU_HASH: return "SHT_GNU_HASH";
    case SHT_GNU_LIBLIST: return "SHT_GNU_LIBLIST";
    case SHT_CHECKSUM: return "SHT_CHECKSUM";
    case SHT_LOSUNW: return "SHT_LOSUNW";
    /*case SHT_SUNW_move: return "SHT_SUNW_move";*/
    case SHT_SUNW_COMDAT: return "SHT_SUNW_COMDAT";
    case SHT_SUNW_syminfo: return "SHT_SUNW_syminfo";
    case SHT_GNU_verdef: return "SHT_GNU_verdef";
    case SHT_GNU_verneed: return "SHT_GNU_verneed";
    case SHT_GNU_versym: return "SHT_GNU_versym";
    /*case SHT_HISUNW: return "SHT_HISUNW";*/
    /*case SHT_HIOS: return "SHT_HIOS";*/
    case SHT_LOPROC: return "SHT_LOPROC";
    case SHT_HIPROC: return "SHT_HIPROC";
    case SHT_LOUSER: return "SHT_LOUSER";
    case SHT_HIUSER: return "SHT_HIUSER";
    default: return NULL;
    }
}
#endif

static __noinstrument
int parse_bytes(char *bytes)
{
    const unsigned char magic[] = { ELFMAG0, ELFMAG1, ELFMAG2, ELFMAG3 };
    Elf64_Ehdr *elf_hdr;
    size_t str_off = 0, sym_off = 0, sym_sz = 0;
    size_t dynstr_off = 0, dynsym_off = 0, dynsym_sz = 0;
    int rc = 0;

    elf_hdr = (Elf64_Ehdr *) bytes;

    if (memcmp(elf_hdr->e_ident, magic, sizeof(magic)) != 0) {
        fprintf(stderr, "Not an ELF executable\n");
        rc = -EINVAL;
        goto out;
    }
    if (elf_hdr->e_ident[EI_CLASS] != ELFCLASS64) {
    	fprintf(stderr, "Only ELF-64 is supported.\n");
        rc = -EINVAL;
        goto out;
    }
    if (elf_hdr->e_machine != EM_X86_64) {
    	fprintf(stderr, "Only x86-64 is supported.\n");
        rc = -EINVAL;
        goto out;
    }


    for (uint16_t i = 0; i < elf_hdr->e_shnum; i++) {
        size_t offset;
        Elf64_Shdr *shdr;

        offset = elf_hdr->e_shoff + i * elf_hdr->e_shentsize;
        shdr = (Elf64_Shdr *) (bytes + offset);

        switch (shdr->sh_type) {
        case SHT_SYMTAB:
            DEBUG("found sym table at %zd\n", shdr->sh_offset);
            sym_off = shdr->sh_offset;
            sym_sz = shdr->sh_size;
            break;

        case SHT_STRTAB:
            DEBUG("found string table at %zd\n", shdr->sh_offset);
            if (!dynstr_off)
                dynstr_off = shdr->sh_offset;
            else if (!str_off)
                str_off = shdr->sh_offset;
            break;

        case SHT_DYNSYM:
            DEBUG("found dynsym table at %zd, size %zd\n",
                shdr->sh_offset, shdr->sh_size);
            dynsym_off = shdr->sh_offset;
            dynsym_sz = shdr->sh_size;
            break;

        default:
            break;
        }

        DEBUG("sh_type=%20s sh_size=%7lu sh_offset=%7lu sh_link=%lu sh_entsize=%lu sh_info=%lu\n",
            sht_string(shdr->sh_type), shdr->sh_size, shdr->sh_offset,
            shdr->sh_link, shdr->sh_entsize, shdr->sh_info);
    }

    DEBUG("dynsym_off=%zd dynsym_sz=%zd dynstr_off=%zd\n",
        dynsym_off, dynsym_sz, dynstr_off);
    DEBUG("sym_off=%zd sym_sz=%zd str_off=%zd\n",
        sym_off, sym_sz, str_off);

    rc = parse_symtable((Elf64_Sym *) (bytes + dynsym_off),
            bytes + dynstr_off, dynsym_sz);
    if (rc) {
        fprintf(stderr, "Error parsing dynamic sym table\n");
        goto out;
    }

    rc = parse_symtable((Elf64_Sym *) (bytes + sym_off),
            bytes + str_off, sym_sz);
    if (rc) {
        fprintf(stderr, "Error parsing sym table\n");
        goto out;
    }

out:
    return rc;
}

static __noinstrument
unsigned int hash_from_key_fn(void *k)
{
    unsigned long value = (unsigned long) k;
    char *p = (char *) &value;
    unsigned int hash = 5381;
    char c;
    int i;

    for (i = 0; i < sizeof(value); i++) {
        c = *p++;
        hash = ((hash << 5) + hash) + (unsigned int) c;
    }

    return hash;
}

static __noinstrument
int keys_equal_fn(void *key1, void *key2)
{
    return ((unsigned long) key1 == (unsigned long) key2);
}

__noinstrument
struct hashtable *trace_create_hashmap(void)
{
	return create_hashtable(16, hash_from_key_fn, keys_equal_fn);
}

__noinstrument int
func_map_init(const char *filename)
{
    char *bytes = NULL;
    long bytes_num = 0;
    int rc = 0;

    func_map = trace_create_hashmap();
    if (!func_map) {
        rc = -ENOMEM;
        fprintf(stderr, "Error calling create_hashtable()\n");
        goto out;
    }

    rc = map_elf_file(filename, (void **) &bytes, &bytes_num);
    if (rc) {
        fprintf(stderr, "Error mapping ELF file\n");
        goto out;
    }

    rc = parse_bytes(bytes);
    if (rc) {
        fprintf(stderr, "Error parsing ELF file\n");
        goto out;
    }

out:
    if (rc) {
        if (func_map) {
            hashtable_destroy(func_map, 1);
            func_map = NULL;
        }
        if (bytes)
            munmap(bytes, bytes_num);
    }

    return rc;
}

__noinstrument const char *
func_map_get(unsigned long address)
{
    const char *func_name;

    func_name = hashtable_search(func_map, (void *) address);

    return func_name;
}

#if TEST_ELF_PARSER
int main(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: %s <elf-binary>\n", argv[0]);
        return 1;
    }

    return func_map_init(argv[1]);
}
#endif
