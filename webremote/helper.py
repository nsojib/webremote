import glob

print('printing files in current directory')
for filename in glob.iglob('inner_dir/**/*', recursive=True):
    print (filename)
    

print('-------')

def main():
    print('main')

if __name__ == '__main__':
    main()

