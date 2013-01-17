" A simple syntax file for the ors file format
" ============================================
" author: stefan.otte@gmail.com
"
" Usage:
"
" 1. cp ors.vim ~/.vim/syntax/
" 2. Add the following to ~/.vimrc:
"
" augroup filetype
"   au! BufRead,BufNewFile *.ors setfiletype ors
" augroup end
"
" Or just create a new file called ~/.vim/ftdetect/ors.vim with the
" previous lines on it.

if version < 600
  syntax clear
elseif exists("b:current_syntax")
  finish
endif

syn case match

" DEFINE keywords matches and regions
" TODO add TODO as keyword :)
" syn keyword orsTodo       contained TODO FIXME XXX
syn keyword orsJoint joint
syn keyword orsBody body
syn keyword orsShape shape
syn region  orsComment start="#" skip="\\$" end="$"
syn match   orsInt     /-\?\<\d\+\>/
syn match   orsInt     /\<0[xX]\x+\>/
syn match   orsFloat   /\<-\?\d*\(\.\d*\)\?/
syn region  orsString  start=/"/ skip=/\\"/ end=/"/
syn region  orsString  start=/'/ skip=/\\'/ end=/'/


" COLORIZE the defined stuff
hi def link  orsJoint        Keyword
hi def link  orsBody         Keyword
hi def link  orsShape        Keyword

hi def link  orsInt          Number
hi def link  orsFloat        Float

hi def link  orsComment      Comment

hi def link  orsString        String

let b:current_syntax = "ors"
