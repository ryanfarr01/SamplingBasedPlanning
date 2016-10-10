% hermans-hw.cls
% variation of jstrom-short.cls from Joho Strom
% inspired by resume.cls from dburrows
%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{hermans-hw}

\LoadClassWithOptions{article}

  \RequirePackage{calc}            % Needed for settoheight manipulations

\usepackage{graphicx}    % needed for including graphics e.g. EPS, PS
\topmargin -1.5cm        % read Lamport p.163
\oddsidemargin -0.04cm   % read Lamport p.163
\evensidemargin -0.04cm  % same as oddsidemargin but for left-hand pages
\textwidth 16.59cm
\textheight 21.94cm
\parskip 7.2pt           % sets spacing between paragraphs
\renewcommand{\baselinestretch}{1.0} % 1.0 (was 1.5) spacing between lines
\parindent 0pt		  % sets leading space for paragraphs
\pagenumbering{arabic}
\usepackage{listings}
\usepackage{ifthen}
\usepackage{ragged2e}
\RaggedRight

  %%%%%%%%%%%%%%%%% define variables for the title
  \newcommand{\@authorname}{}
  \renewcommand{\@title}{}
  \newcommand{\@class}{}
  \newcommand{\@duedate}{}
  \newcommand{\@institute}{}

  \renewcommand{\author}[1]{\renewcommand{\@authorname}{#1}}
  \renewcommand{\title}[1]{\renewcommand{\@title}{#1}}
  \newcommand{\class}[1]{\renewcommand{\@class}{#1}}
  \newcommand{\duedate}[1]{\renewcommand{\@duedate}{#1}}
  \newcommand{\institute}[1]{\renewcommand{\@institute}{#1}}

% TODO: Make writing of second line conditional on \duedate and \class being defined
% TODO: Same for institute
  %%%%% Define a table to hold the author, etc to be placed in title
  \newcommand{\authortable}{
    \begin{tabular}{@{} r @{}}
      \@authorname \\
      \@duedate\\
    \end{tabular}
  }

  %%%%% Over ride the \maketitle command %%%%%
  \renewcommand{\maketitle}{
    \par
    %%%%% Discover the size of the author box so we can
    %%%%% give the correct amount of space to the title
    \newlength{\authorwidth}
    \settowidth{\authorwidth}{\authortable}
    %%%%% Place the title and the author box side-by-side
    \noindent
    \parbox{
      (\textwidth)}{\bf{\Large \@title} \\
      \@class \hfill  \@authorname\\
      \@institute \hfill \@duedate
    }  \newline
    % \mbox{ \authortable }
    %%%%% Find height of the current font
    \newlength{\fontheight}
    \settoheight{\fontheight}{A}
    %%%%% Draw a horizontal line
    \noindent \rule[\fontheight]{\textwidth}{.5pt}
  }
