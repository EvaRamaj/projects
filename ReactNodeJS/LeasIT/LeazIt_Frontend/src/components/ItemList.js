"use strict";

import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';

import { ItemListRow } from './ItemListRow';
import Page from './Page'


export const ItemList = ({data, full_rows, extra_cols}) => (

    <Page>
        {/*<DataTable plain>*/}
            {/*<TableHeader>*/}
                {/*<TableRow>*/}
                    {/*<TableColumn></TableColumn>*/}
                    {/*<TableColumn>Name</TableColumn>*/}
                {/*</TableRow>*/}
            {/*</TableHeader>*/}
            {/*<TableBody>*/}
                {data.map((items, i) => <ItemListRow key={i} items={items}  />)}
            {/*</TableBody>*/}
        {/*</DataTable>*/}
    </Page>
);

