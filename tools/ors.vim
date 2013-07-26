" A simple syntax file for the ors file format
" ============================================
" author: stefan.otte@gmail.com
"
" Set the following in your .vimrc to automatically load the ors syntax file.
"
" augroup ft_ors
"     au!
"     autocmd BufRead,BufNewFile *.ors setlocal filetype=ors
" augroup END

" INIT {{{
if version < 600
  syntax clear
elseif exists("b:current_syntax")
  finish
endif

syn case match
" }}}
" DEFINE keywords matches and regions {{{
syn keyword orsJoint joint nextgroup=orsJointName skipwhite
syn match orsJointName "([a-zA-Z_][a-zA-Z0-9_-]*\s*,\s*[a-zA-Z_][a-zA-Z0-9_-]*)" display contained

syn keyword orsBody body nextgroup=orsBodyName skipwhite
syn match orsBodyName "[a-zA-Z_][a-zA-Z0-9_-]*" display contained

syn keyword orsShape shape nextgroup=orsShapeName skipwhite
syn match orsShapeName "[a-zA-Z_][a-zA-Z0-9_-]*" display contained

syn match   orsInt     /-\?\<\d\+\>/
syn match   orsInt     /\<0[xX]\x+\>/
syn match   orsFloat   /\<-\?\d*\(\.\d*\)\?/
syn region  orsString  start=/"/ skip=/\\"/ end=/"/
syn region  orsString  start=/'/ skip=/\\'/ end=/'/

syn match  orsComment "//.*$" display contains=orsTodo
syn keyword orsTodo       contained TODO FIXME XXX
" }}}
" COLORIZE the defined stuff {{{
hi def link  orsJoint        Keyword
hi def link  orsJointName    Function
hi def link  orsBody         Keyword
hi def link  orsBodyName     Function
hi def link  orsShape        Keyword
hi def link  orsShapeName    Function

hi def link  orsInt          Number
hi def link  orsFloat        Float
hi def link  orsString        String

hi def link  orsComment      Comment
hi def link  orsTodo Todo
" }}}
let b:current_syntax = "ors"
