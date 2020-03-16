"use strict";

import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';

import { SearchListRow } from './SearchListRow';
import Page from './Page'



export const SearchList = ({keywords,data,user_evals}) => (

    <Page>
        {/*<DataTable plain>*/}
            {/*<TableHeader>*/}
                {/*<TableRow>*/}
                    {/*<TableColumn></TableColumn>*/}
                    {/*<TableColumn>Name</TableColumn>*/}
                {/*</TableRow>*/}
            {/*</TableHeader>*/}
            {/*<TableBody>*/}
            <p>Search results for keyword(s): {keywords}</p>
            <div class="row">
                {data.map((items, i) => <SearchListRow key={i} items={items} user_evals = {user_evals} />)}
            </div>
            {/*</TableBody>*/}
        {/*</DataTable>*/}
    </Page>
);

