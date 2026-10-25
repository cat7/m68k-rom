m68k_ss = ss.source_set()
m68k_ss.add(when: 'CONFIG_AN5206', if_true: files('an5206.c', 'mcf5206.c'))
m68k_ss.add(when: 'CONFIG_MCF5208', if_true: files('mcf5208.c', 'mcf_intc.c'))
m68k_ss.add(when: 'CONFIG_NEXTCUBE', if_true: files('next-kbd.c', 'next-cube.c'))
m68k_ss.add(when: 'CONFIG_MAC_QUADRA', if_true: files('mac-quadra.c', 'q800-glue.c'))
m68k_ss.add(when: 'CONFIG_Q800', if_true: files('q800.c'))
m68k_ss.add(when: 'CONFIG_Q700', if_true: files('q700.c'))
m68k_ss.add(when: 'CONFIG_Q900', if_true: files('q900.c'))
m68k_ss.add(when: 'CONFIG_LC475', if_true: files('lc475.c'))
m68k_ss.add(when: 'CONFIG_M68K_VIRT', if_true: files('virt.c'))

hw_arch += {'m68k': m68k_ss}
